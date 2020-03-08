#include"contactConstraint.h"

#include"physics/physDefine.h"
#include"core/error.h"
#include<algorithm>

using ntw::crossProduct;
using std::min;
using std::max;


ContactConstraint::ContactConstraint(Contact& contact, ObjectPair& objects) :
	Constraint(objects.object1, objects.object2, true), contact_(contact) {

}

void ContactConstraint::init(){

	Constraint::init();

	if(firstSolve_){
		// Initialize jacobians
		jacNormal_ = jacTangent1_ = jacTangent2_ = jac_;

		// Normal constraint
		Vec3 vel1		= object1_->getPhysicsType() == PhysicsType::DYNAMIC ? ((PhysicsObject*)object1_)->getVelocity()		: Vec3();
		Vec3 angVel1	= object1_->getPhysicsType() == PhysicsType::DYNAMIC ? ((PhysicsObject*)object1_)->getAngularVelocity()	: Vec3();
		Vec3 vel2		= object2_->getPhysicsType() == PhysicsType::DYNAMIC ? ((PhysicsObject*)object2_)->getVelocity()		: Vec3();
		Vec3 angVel2	= object2_->getPhysicsType() == PhysicsType::DYNAMIC ? ((PhysicsObject*)object2_)->getAngularVelocity()	: Vec3();

		// Baumgarte stabilization
		biasNormal_ = -(PHYS_BAUMGARTE_FAC / PHYS_TIMESTEP) * max(contact_.penetrationDepth - PHYS_PENETRATION_SLOP, 0.0f);

		// Restitution
		// TODO: change multiplier to factor determined by material elasticity
		contact_.closingSpeed = ((-vel1 - crossProduct(angVel1, contact_.obj1ContactVector)
			+ (vel2 + crossProduct(angVel2, contact_.obj2ContactVector))) * -contact_.normal);
		biasNormal_ += 0.5f * max(contact_.closingSpeed - PHYS_RESTITUTION_SLOP, 0.0f);
	}

	// Set jacobians
	auto l_setJacobian = [](Matrix& jac, const Contact& contact, const Vec3& direction) -> void {
		jac.place(0, 0, -direction, true);
		jac.place(0, 3, -crossProduct(contact.obj1ContactVector, direction), true);
		jac.place(0, 6, direction, true);
		jac.place(0, 9, crossProduct(contact.obj2ContactVector, direction), true);
	};

	l_setJacobian(jacNormal_, contact_, -contact_.normal);
	l_setJacobian(jacTangent1_, contact_, contact_.tangent1);
	l_setJacobian(jacTangent2_, contact_, contact_.tangent2);
}

// Set constraint properties for each constraint direction
void ContactConstraint::setProperties(int type){

	// Normal direction
	if(type == 0){
		jac_ = jacNormal_;
		bias_ = biasNormal_;
		constrainGreaterThanZero_ = true;
		return;
	}

	// Tangent directions
	else{
		jac_ = type == 1 ? jacTangent1_ : jacTangent2_;
		bias_ = 0;
		constrainGreaterThanZero_ = false;
		return;
	}
}

bool ContactConstraint::solve(){

	init();

	// Check that all 3 constraints are solved
	bool solved = true;

	// For each constraint direction
	for(int i = 0; i < 3; i++){

		// Set constraint properties
		setProperties(i);

		// Calculate individual constraint value and check if it should be enforced
		if(calcConstraint())
			continue;


		calcInvMassJt();

		// If this is the first solve and the contact is persistent, use previous lambda sum as current lambda
		if(firstSolve_ && !contact_.isNew){
			switch(i){
			case 0:	lambda_ = contact_.lambdaSum		* PHYS_WARM_START_LAMBDA_MULTIPLIER;	break;
			case 1:	lambda_ = contact_.lambdaSumTan1	* PHYS_WARM_START_LAMBDA_MULTIPLIER;	break;
			case 2:	lambda_ = contact_.lambdaSumTan2	* PHYS_WARM_START_LAMBDA_MULTIPLIER;	break;
			}
		}

		// Calculate current lambda
		else
			calcLambda();

		// Get lambda sum according to constraint type
		float* lambdaSum = &contact_.lambdaSum;
		switch(i){
		case 1:	lambdaSum = &contact_.lambdaSumTan1;	break;
		case 2:	lambdaSum = &contact_.lambdaSumTan2;	break;
		}

		// Copy and add to sum
		float lambdaSumCopy = *lambdaSum;
		*lambdaSum += lambda_;

		// Clamp according to constraint type
		if(i == 0)
			*lambdaSum = max(*lambdaSum, 0.0f);
		else{
			// TODO: change multiplier to coefficient of friction
			float clamp = 0.5f * contact_.lambdaSum;
			*lambdaSum = max(*lambdaSum, -clamp);
			*lambdaSum = min(*lambdaSum, clamp);
		}

		// Compute actual lambda
		lambda_ = *lambdaSum - lambdaSumCopy;

		// If lambda is small enough, it will not have much effect so consider the constraint solved
		if(abs(lambda_) < PHYS_CONSTRAINT_THRESHOLD)
			continue;


		// Add lambda to average
		auto l_addToAvg = [](float& avg, float val, int times) -> void {
			avg = ((avg * times) + val) / (times + 1);
		};

		switch(i){
		case 0:	l_addToAvg(contact_.lambdaAvg,		lambda_, contact_.numSolves);	break;
		case 1:	l_addToAvg(contact_.lambdaAvgTan1,	lambda_, contact_.numSolves);	break;
		case 2:	l_addToAvg(contact_.lambdaAvgTan2,	lambda_, contact_.numSolves);	break;
		}


		// Solve and apply
		calcVelCor();
		apply();

		solved &= calcConstraint();
	}

	firstSolve_ = false;
	contact_.numSolves++;

	return solved;
}

// Necessary for some reason
ContactConstraint& ContactConstraint::operator=(const ContactConstraint& a){
	contact_ = a.getContact();

	return *this;
}

Contact& ContactConstraint::getContact() const{
	return contact_;
}

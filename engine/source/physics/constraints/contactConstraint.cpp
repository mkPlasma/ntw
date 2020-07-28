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

	// Initialize jacobians
	if(firstSolve_)
		jacNormal_ = jacTangent1_ = jacTangent2_ = jac_;
	
	// Recalculate contact vectors
	contact_.obj1ContactVector = contact_.obj1ContactGlobal - object1_->getTPosition();
	contact_.obj2ContactVector = contact_.obj2ContactGlobal - object2_->getTPosition();

	bool obj1Phys = object1_->getPhysicsType() == PhysicsType::RIGID_BODY || object1_->getPhysicsType() == PhysicsType::SIMPLE;
	bool obj2Phys = object2_->getPhysicsType() == PhysicsType::RIGID_BODY || object2_->getPhysicsType() == PhysicsType::SIMPLE;

	// Normal constraint
	Vec3 vel1		= obj1Phys ? ((PhysicsObject*)object1_)->getVelocity()			: Vec3();
	Vec3 angVel1	= obj1Phys ? ((PhysicsObject*)object1_)->getAngularVelocity()	: Vec3();
	Vec3 vel2		= obj2Phys ? ((PhysicsObject*)object2_)->getVelocity()			: Vec3();
	Vec3 angVel2	= obj2Phys ? ((PhysicsObject*)object2_)->getAngularVelocity()	: Vec3();

	// Baumgarte stabilization
	biasNormal_ = -(NTW_PHYS_BAUMGARTE_FAC / NTW_PHYS_TIME_DELTA) * max(contact_.depth - NTW_PHYS_PENETRATION_SLOP, 0.0f);

	// Restitution
	contact_.closingSpeed = ((-vel1 - crossProduct(angVel1, contact_.obj1ContactVector)
		+ (vel2 + crossProduct(angVel2, contact_.obj2ContactVector))) * -contact_.normal);
	// TODO: change multiplier (0.5f) to factor determined by material elasticity
	biasNormal_ += 0.1f * max(contact_.closingSpeed - NTW_PHYS_RESTITUTION_SLOP, 0.0f);

	// Set jacobians
	auto l_setJacobian = [](Matrix& jac, const Contact& contact, const Vec3& direction) -> void {
		jac.place(0, 0, -direction, true);
		jac.place(0, 3, crossProduct(-contact.obj1ContactVector, direction), true);
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

void ContactConstraint::solve(){

	init();

	// For each constraint direction
	for(int i = 0; i < 3; i++){

		// Set constraint properties
		setProperties(i);

		// Calculate individual constraint value and check if it should be enforced
		if(calcConstraint())
			continue;


		calcInvMassJt();

		// If this is the first solve and the contact is persistent, use previous lambda sum as current lambda
		/*
		if(firstSolve_ && !contact_.isNew){
			switch(i){
			case 0:	lambda_ = contact_.lambdaSum		* PHYS_WARM_START_LAMBDA_MULTIPLIER;	break;
			case 1:	lambda_ = contact_.lambdaSumTan1	* PHYS_WARM_START_LAMBDA_MULTIPLIER;	break;
			case 2:	lambda_ = contact_.lambdaSumTan2	* PHYS_WARM_START_LAMBDA_MULTIPLIER;	break;
			}
		}
		*/
		// Calculate current lambda
		//else
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
		if(abs(lambda_) < NTW_PHYS_CONSTRAINT_THRESHOLD)
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
	}

	firstSolve_ = false;
	contact_.numSolves++;
}

bool ContactConstraint::isSolved(){

	init();

	// Check that all 3 constraints are solved
	for(int i = 0; i < 3; i++){
		setProperties(i);

		if(!Constraint::isSolved())
			return false;
	}

	return true;
}

// Necessary for some reason
ContactConstraint& ContactConstraint::operator=(const ContactConstraint& a){
	contact_ = a.getContactPoints();

	return *this;
}

Contact& ContactConstraint::getContactPoints() const{
	return contact_;
}

#include"contactConstraint.h"

#include"physics/physDefine.h"
#include"core/error.h"
#include<algorithm>

using ntw::crossProduct;
using std::min;
using std::max;


ContactConstraint::ContactConstraint(Contact& contact, ObjectPair& objects) :
	Constraint(objects.object1, objects.object2, true), contact_(contact), firstSolve_(true) {

	init();

	// Initialize jacobians
	jacNormal_ = jacTangent1_ = jacTangent2_ = jac_;

	// Normal constraint
	Vec3 penetrationNormal = -contact.penetration.unitVector();

	Vec3 vel1		= object1_->getPhysicsType() == PHYS_DYNAMIC ? ((PhysicsObject*)object1_)->getVelocity()		: Vec3();
	Vec3 angVel1	= object1_->getPhysicsType() == PHYS_DYNAMIC ? ((PhysicsObject*)object1_)->getAngularVelocity()	: Vec3();
	Vec3 vel2		= object2_->getPhysicsType() == PHYS_DYNAMIC ? ((PhysicsObject*)object2_)->getVelocity()		: Vec3();
	Vec3 angVel2	= object2_->getPhysicsType() == PHYS_DYNAMIC ? ((PhysicsObject*)object2_)->getAngularVelocity()	: Vec3();

	// Baumgarte stabilization
	float penetrationDepth = (contact.obj2ContactGlobal - contact.obj1ContactGlobal) * -penetrationNormal;
	biasNormal_ = -(PHYS_BAUMGARTE_FAC / PHYS_TIMESTEP) * max(penetrationDepth - PHYS_PENETRATION_SLOP, 0.0f);

	// Restitution
	// TODO: change multiplier to factor determined by material elasticity
	contact.closingSpeed = ((-vel1 - crossProduct(angVel1, contact.obj1ContactVector)
		+ (vel2 + crossProduct(angVel2, contact.obj2ContactVector))) * penetrationNormal);
	biasNormal_ += 0.5f * max(contact.closingSpeed - PHYS_RESTITUTION_SLOP, 0.0f);

	// Set jacobians
	auto l_setJacobian = [](Matrix& jac, const Contact& contact, const Vec3& direction) -> void {
		jac.place(0, 0, -direction, true);
		jac.place(0, 3, crossProduct(-contact.obj1ContactVector, direction), true);
		jac.place(0, 6, direction, true);
		jac.place(0, 9, crossProduct(contact.obj2ContactVector, direction), true);
	};

	l_setJacobian(jacNormal_,	contact, penetrationNormal);
	l_setJacobian(jacTangent1_,	contact, contact.tangent1);
	l_setJacobian(jacTangent2_,	contact, contact.tangent2);
}

// Set constraint properties for each constraint direction
void ContactConstraint::setProperties(int type){

	// Normal direction
	if(type == 0){
		jac_ = jacNormal_;
		bias_ = biasNormal_;
		constrainGreaterThanZero_ = firstSolve_;
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

// Check all 3 constraint directions
inline bool ContactConstraint::calcConstraint(){
	for(int i = 0; i < 3; i++){

		setProperties(i);

		if(!Constraint::calcConstraint())
			return false;
	}

	return true;
}

void ContactConstraint::solve(){

	// For each constraint direction
	for(int i = 0; i < 1; i++){

		// Set constraint properties
		setProperties(i);

		// Calculate individual constraint value and check if it should be enforced
		if(Constraint::calcConstraint())
			return;

		calcInvMassJt();

		// If this is the first solve and the contact is persistent,
		// use its lambda sum as the current lambda
		if(firstSolve_ && contact_.persistent){
			switch(i){
			case 0:	lambda_ = contact_.lambdaSum		* PHYS_WARM_START_LAMBDA_MULTIPLIER;	break;
			case 1:	lambda_ = contact_.lambdaSumTan1	* PHYS_WARM_START_LAMBDA_MULTIPLIER;	break;
			case 2:	lambda_ = contact_.lambdaSumTan2	* PHYS_WARM_START_LAMBDA_MULTIPLIER;	break;
			}
		}

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
			float val = 0.5f * contact_.lambdaSum;

			if(*lambdaSum > val)
				*lambdaSum = val;
			else if(*lambdaSum < -val)
				*lambdaSum = -val;
		}

		// Compute actual lambda
		lambda_ = *lambdaSum - lambdaSumCopy;

		// Solve and apply
		calcVelCor();
		apply();
	}

	firstSolve_ = false;
}

// Necessary for some reason
ContactConstraint& ContactConstraint::operator=(const ContactConstraint& a){
	contact_ = a.getContact();

	return *this;
}

Contact& ContactConstraint::getContact() const{
	return contact_;
}

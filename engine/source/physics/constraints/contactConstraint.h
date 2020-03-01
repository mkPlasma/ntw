#pragma once

/*
 *	contactConstraint.h
 *
 *	Non-penetration and friction impulse constraint.
 *
 */

#include"constraint.h"


class ContactConstraint : public Constraint{

	Contact& contact_;

	Matrix jacNormal_;
	Matrix jacTangent1_;
	Matrix jacTangent2_;

	float biasNormal_;

	bool firstSolve_;

	void setProperties(int type);

public:
	ContactConstraint(Contact& contact, ObjectPair& objects);

	inline bool calcConstraint() override;
	void solve() override;

	// Necessary for some reason
	ContactConstraint& operator=(const ContactConstraint& a);

	Contact& getContact() const;
};

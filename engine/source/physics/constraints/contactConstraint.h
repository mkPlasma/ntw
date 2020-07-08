#pragma once

/*
 *	contactConstraint.h
 *
 *	Non-penetration and friction impulse constraint.
 *
 */

class ContactConstraint;

#include"constraint.h"


class ContactConstraint : public Constraint{

	Contact& contact_;

	Matrix jacNormal_;
	Matrix jacTangent1_;
	Matrix jacTangent2_;

	float biasNormal_;

	void setProperties(int type);

public:
	ContactConstraint(Contact& contact, ObjectPair& objects);

	void init() override;
	void solve() override;

	bool isSolved() override;

	// Necessary for some reason
	ContactConstraint& operator=(const ContactConstraint& a);

	Contact& getContactPoints() const;
};

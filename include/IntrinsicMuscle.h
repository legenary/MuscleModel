#ifndef INTRINSIC_MUSCLE_H
#define INTRINSIC_MUSCLE_H

#include "Spring.h"
#include "Utility.h"
#include "Follicle.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Intrinsic sling muscle
class IntrinsicSlingMuscle : public Spring {
private:
	void init(btScalar k, btScalar damping);

public:
	IntrinsicSlingMuscle(btRigidBody* body1, btRigidBody* body2, btTransform frameInParent, btTransform frameInChild, btScalar k, btScalar damping=0.01);
	virtual ~IntrinsicSlingMuscle(){}

	void contract(btScalar ratio);

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Intrinsic oblique muscle
class IntrinsicObliqueMuscle : public Spring {
private:
	void init(btScalar k, btScalar damping);

public:
	IntrinsicObliqueMuscle(btRigidBody* body1, btRigidBody* body2, btTransform frameInParent, btTransform frameInChild, btScalar k, btScalar damping=0.01);
	virtual ~IntrinsicObliqueMuscle() {}

	void contract();

};






#endif
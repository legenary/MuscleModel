#ifndef INTRINSIC_MUSCLE_H
#define INTRINSIC_MUSCLE_H

#include "Tissue.h"

class Follicle;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Intrinsic sling muscle
class IntrinsicSlingMuscle : public Tissue {
private:

	// parameters for muscle contraction


public:
	IntrinsicSlingMuscle(btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping=0.01);
	virtual ~IntrinsicSlingMuscle() {};

	void contract(btScalar ratio);
	void contract_HillType();

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Intrinsic oblique muscle
class IntrinsicObliqueMuscle : public Tissue {
private:

public:
	IntrinsicObliqueMuscle(btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping=0.01);
	virtual ~IntrinsicObliqueMuscle() {}

	void contract();

};






#endif
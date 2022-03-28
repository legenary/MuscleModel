#ifndef INTRINSIC_MUSCLE_H
#define INTRINSIC_MUSCLE_H

#include "Fiber.h"
#include "Simulation.h"

class Follicle;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Intrinsic sling muscle
class IntrinsicSlingMuscle : public Fiber {
private:

	// parameters for muscle contraction


public:
	IntrinsicSlingMuscle(Simulation* sim, btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping=0.01);
	virtual ~IntrinsicSlingMuscle() {};

	//void contractTo(btScalar ratio);

};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Intrinsic oblique muscle
class IntrinsicObliqueMuscle : public Fiber {
private:

public:
	IntrinsicObliqueMuscle(Simulation* sim, btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping=0.01);
	virtual ~IntrinsicObliqueMuscle() {}

	void contract();

};






#endif
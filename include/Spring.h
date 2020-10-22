#ifndef SPRING_H
#define SPRING_H

#include "Utility.h"
#include "Follicle.h"

class Spring {
protected:
	btGeneric6DofSpringConstraint* constraint;
	Follicle* follicle1;
	Follicle* follicle2;
	btTransform frameInBody1;
	btTransform frameInBody2;
	btTransform frameInWorld1;
	btTransform frameInWorld2;
	btVector3 eq;	// equilibrium point that is restLength from body 1 frame
					// equilibrium point is represented in world frame
	btScalar restLength;
	btScalar length;

public:
	Spring(Follicle* fol1, Follicle* fol2, btTransform frameInParent, btTransform frameInChild, btScalar k, btScalar damping = 0.01);
	virtual ~Spring(){}
	
	void update();
	btGeneric6DofSpringConstraint* getConstraint();
	btScalar getRestLength();
	btScalar getLength();

	void debugDraw(btDiscreteDynamicsWorld* world, btVector3 clr = btVector3(0., 0., 0.));
};



#endif
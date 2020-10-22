#ifndef SPRING_H
#define SPRING_H

#include "Utility.h"
#include "Follicle.h"

class Spring {
private:
	btGeneric6DofSpringConstraint* constraint;
	Follicle* follicle1;
	Follicle* follicle2;
	btTransform frameInBody1;
	btTransform frameInBody2;
	btScalar restLength;

public:
	Spring(Follicle* fol1, Follicle* fol2, btTransform frameInParent, btTransform frameInChild, btScalar k);
	virtual ~Spring(){}
	
	void update();
	btGeneric6DofSpringConstraint* getConstraint();

	void test();
};



#endif
#ifndef SPRING_H
#define SPRING_H


class Spring {
protected:
	btGeneric6DofSpringConstraint* constraint;
	btRigidBody* body1;
	btRigidBody* body2;
	btTransform T1P;
	btTransform T2Q;
	btTransform TsP;
	btTransform TsQ;
	btVector3 eq;	// equilibrium point that is restLength from body 1 frame
					// equilibrium point is represented in world frame
	btScalar restLength;
	btScalar restLengthDefault;
	btScalar length;

public:
	Spring(btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild);
	Spring(btRigidBody* body2, btTransform& frameInChild, btScalar k, btScalar damping = 0.01);
	virtual ~Spring(){}

	// "initialize" is a place-holder function that needs to be replaced with specialized initializing function, 
	// depending on type of spring (collagen, muscle...)
	// this function should be removed in future version.
	void initialize(btScalar k, btScalar damping = 0.01, bool isLinear = true);
	void initializeLayer(btScalar E_skin, btScalar m1, btScalar m2);
	
	void update();
	btGeneric6DofSpringConstraint* getConstraint();
	btScalar getRestLength();
	void setRestLength(btScalar ratio);
	btScalar getLength();

	void debugDraw(btDiscreteDynamicsWorld* world, btVector3 clr = btVector3(0., 0., 0.));
};


#endif
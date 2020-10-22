#include "Spring.h"

Spring::Spring(Follicle* fol1, Follicle* fol2, btTransform frame1, btTransform frame2, btScalar k) {

	follicle1 = fol1;
	follicle2 = fol2;
	frameInBody1 = frame1;
	frameInBody2 = frame2;

	btRigidBody* body1 = fol1->getBody();
	btRigidBody* body2 = fol2->getBody();
	constraint = new btGeneric6DofSpringConstraint(*body1, *body2, frameInBody1, frameInBody2, true);
	
	constraint->setLinearLowerLimit(btVector3(1, 1, 1));	// need to set lower > higher to free the dofs
	constraint->setLinearUpperLimit(btVector3(0, 0, 0));
	
	for (int i = 0; i < 3; i++) {
		constraint->enableSpring(i, true);
		constraint->setStiffness(i, k);
		constraint->setDamping(i, 1);	// guess: damping [0, 1] like restitution coefficient?
		constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
	}
	restLength = btVector3(constraint->getEquilibriumPoint(0),
						   constraint->getEquilibriumPoint(1),
						   constraint->getEquilibriumPoint(2)).length();
}

// NEED DEBUG
void Spring::update() {
	//// update equilibrium point location in frame 1
	//// First, get two attachment points location in world reference frame
	//btTransform P = follicle1->getBody()->getCenterOfMassTransform();
	//P *= frameInBody1;
	//btVector3 p = P.getOrigin();
	//btTransform Q = follicle2->getBody()->getCenterOfMassTransform();
	//Q *= frameInBody2;
	//btVector3 q = Q.getOrigin();
	//// Second, get the equilibrium point location in world reference frame
	//btVector3 eq = q + (p - q)*restLength / (p - q).length();
	//// Third, get the equilibruim point location in body1 reference frame
	//btVector3 eq1 = P.inverse()*eq;

	//for (int i = 0; i < 3; i++) {
	//	constraint->setEquilibriumPoint(i, eq1[i]);
	//}
}

void Spring::test() {
	btTransform P = follicle1->getBody()->getCenterOfMassTransform();
	std::cout << P.getOrigin()[0];

	std::cout << frameInBody1.getOrigin()[0] << " " << frameInBody1.getOrigin()[1] << " " << frameInBody1.getOrigin()[2];
}

btGeneric6DofSpringConstraint* Spring::getConstraint() {
	return constraint;
}
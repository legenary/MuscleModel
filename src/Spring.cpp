#include "Spring.h"

Spring::Spring(Follicle* fol1, Follicle* fol2, btTransform frame1, btTransform frame2, btScalar k, btScalar damping) {

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
		constraint->setDamping(i, damping);	// guess: damping [0, 1] like restitution coefficient?
		constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
	}
	restLength = btVector3(constraint->getEquilibriumPoint(0),
						   constraint->getEquilibriumPoint(1),
						   constraint->getEquilibriumPoint(2)).length();
	length = restLength;
}

// NEED DEBUG
void Spring::update() {
	// update equilibrium point location in frame 1
	// First, get two attachment points location in world reference frame
	frameInWorld1 = follicle1->getBody()->getCenterOfMassTransform();
	frameInWorld1 *= frameInBody1;
	frameInWorld2 = follicle2->getBody()->getCenterOfMassTransform();
	frameInWorld2 *= frameInBody2;
	btVector3 p = frameInWorld1.getOrigin();
	btVector3 q = frameInWorld2.getOrigin();
	// Second, get the equilibrium point location in world reference frame
	length = (q - p).length();
	eq = p + (q - p)*restLength/length;
	// Third, get the equilibruim point location in body1 reference frame
	btVector3 eq_in_p = frameInWorld1.inverse()*eq;

	for (int i = 0; i < 3; i++) {
		constraint->setEquilibriumPoint(i, eq_in_p[i]);
	}
}

void Spring::debugDraw(btDiscreteDynamicsWorld* m_dynamicsWorld, btVector3 clr) {
	m_dynamicsWorld->getDebugDrawer()->drawLine(frameInWorld1.getOrigin(), eq, clr);
}

btScalar Spring::getRestLength() {
	return restLength;
}

btScalar Spring::getLength() {
	return length;
}

btGeneric6DofSpringConstraint* Spring::getConstraint() {
	return constraint;
}
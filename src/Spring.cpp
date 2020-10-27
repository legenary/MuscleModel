#include "Spring.h"

Spring::Spring(Follicle* fol1, Follicle* fol2, btTransform frame1, btTransform frame2, btScalar k, btScalar damping, bool isLinear) {

	follicle1 = fol1;
	follicle2 = fol2;
	T1P = frame1;
	T2Q = frame2;

	btRigidBody* body1 = fol1->getBody();
	btRigidBody* body2 = fol2->getBody();
	constraint = new btGeneric6DofSpringConstraint(*body1, *body2, T1P, T2Q, true);
	
	if (isLinear) {
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
	}
	else {
		std::cout << "In Spring::Spring(Follicle* fol1, Follicle* fol2, btTransform frame1, btTransform frame2, btScalar k, btScalar damping, bool isLinear):" << std::endl;
		std::cout << "Torsional Spring between rigid bodies not implemented. Initialization Failed..." << std::endl;
		restLength = 0;
	}

	length = restLength;
	restLengthDefault = restLength;
}

Spring::Spring(Follicle* fol2, btTransform frame2, btScalar k, btScalar damping, bool isLinear) {

	follicle2 = fol2;
	T1P = createTransform();
	T2Q = frame2;

	btRigidBody* body2 = fol2->getBody();
	constraint = new btGeneric6DofSpringConstraint(*body2, T2Q, true);

	if (isLinear) {
		std::cout << "In Spring::Spring(Follicle* fol2, btTransform frame2, btScalar k, btScalar damping, bool isLinear):" << std::endl;
		std::cout << "Linear Spring between a rigid body and the world not implementd. Initialization Failed..." << std::endl;
		restLength = 0;
	}
	else {
		constraint->setAngularLowerLimit(btVector3(1, 1, 1));	// need to set lower > higher to free the dofs
		constraint->setAngularUpperLimit(btVector3(0, 0, 0));
		for (int i = 0; i < 6; i++) {
			constraint->enableSpring(i, true);
			constraint->setStiffness(i, k);
			constraint->setDamping(i, damping);	// guess: damping [0, 1] like restitution coefficient?
			constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
		}
		restLength = btVector3(constraint->getEquilibriumPoint(0),
			constraint->getEquilibriumPoint(1),
			constraint->getEquilibriumPoint(2)).length();
	}

	length = restLength;
	restLengthDefault = restLength;
}

// NEED DEBUG
void Spring::update() {
	// update equilibrium point location in frame 1
	// First, get two attachment points location in world reference frame
	TsP = follicle1->getBody()->getCenterOfMassTransform();
	TsP *= T1P;
	TsQ = follicle2->getBody()->getCenterOfMassTransform();
	TsQ *= T2Q;
	btVector3 p = TsP.getOrigin();
	btVector3 q = TsQ.getOrigin();
	// Second, get the equilibrium point location in world reference frame
	length = (q - p).length();
	eq = p + (q - p)*restLength/length;
	// Third, get the equilibruim point location in body1 reference frame
	btVector3 eq_in_p = TsP.inverse()*eq;

	for (int i = 0; i < 3; i++) {
		constraint->setEquilibriumPoint(i, eq_in_p[i]);
	}
}

void Spring::debugDraw(btDiscreteDynamicsWorld* m_dynamicsWorld, btVector3 clr) {
	m_dynamicsWorld->getDebugDrawer()->drawLine(TsP.getOrigin(), eq, clr);
}

btScalar Spring::getRestLength() {
	return restLength;
}

void Spring::setRestLength(btScalar ratio) {
	restLength = ratio * restLengthDefault;
}

btScalar Spring::getLength() {
	return length;
}

btGeneric6DofSpringConstraint* Spring::getConstraint() {
	return constraint;
}
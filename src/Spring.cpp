#include "Spring.h"

Spring::Spring(btRigidBody* b1, btRigidBody* b2, btTransform& frame1, btTransform& frame2) {
	// spring between two bodies are linear

	body1 = b1;
	body2 = b2;
	T1P = frame1;
	T2Q = frame2;

	constraint = new btGeneric6DofSpringConstraint(*body1, *body2, T1P, T2Q, true);
}

Spring::Spring(btRigidBody* b2, btTransform& frame2, btScalar k, btScalar damping) {
	// spring for anchoring a body to the world is both linear and torsional

	body2 = b2;
	T1P = createTransform();
	T2Q = frame2;

	constraint = new btGeneric6DofSpringConstraint(*body2, T2Q, true);

	constraint->setLinearLowerLimit(btVector3(k, k, k));	// need to set lower > higher to free the dofs
															// if k = 0, lock the linear movement (switch to hard anchoring mode)
	constraint->setLinearUpperLimit(btVector3(0, 0, 0));
	constraint->setAngularLowerLimit(btVector3(1, 1, 1));	// need to set lower > higher to free the dofs
	constraint->setAngularUpperLimit(btVector3(0, 0, 0));
	for (int i = 0; i < 3; i++) {
		constraint->enableSpring(i, true);
		constraint->setStiffness(i, k);
		constraint->setDamping(i, damping);	// guess: damping [0, 1] like restitution coefficient?
		constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
	}
	for (int i = 3; i < 6; i++) {
		constraint->enableSpring(i, true);
		constraint->setStiffness(i, k/10);
		constraint->setDamping(i, damping);	// guess: damping [0, 1] like restitution coefficient?
		constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
	}
	restLength = btVector3(constraint->getEquilibriumPoint(0),
							constraint->getEquilibriumPoint(1),
							constraint->getEquilibriumPoint(2)).length();

	length = restLength;
	restLengthDefault = restLength;
}

void Spring::initialize(btScalar k, btScalar damping, bool isLinear) {
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
		std::cout << "In Spring::Spring(btRigidBody* b1, btRigidBody* b2, btTransform frame1, btTransform frame2, btScalar k, btScalar damping, bool isLinear):" << std::endl;
		std::cout << "Torsional Spring between rigid bodies not implemented. Initialization Failed..." << std::endl;
		restLength = 0;
	}
	length = restLength;
	restLengthDefault = restLength;
}

void Spring::initializeLayer(btScalar E_skin, btScalar m1, btScalar m2) {
	// this initialization calculate spring constants based on the skin Young's Modulus and follicle interspace.
	// Note: 2 springs in parallel, so springs constants are divided by 2
	//								but damping in Bullet should be multiplied by 2
	
	

	constraint->setLinearLowerLimit(btVector3(1, 1, 1));	// need to set lower > higher to free the dofs
	constraint->setLinearUpperLimit(btVector3(0, 0, 0));
	for (int i = 0; i < 3; i++) {
		constraint->enableSpring(i, true);
		constraint->setStiffness(i, 100);
		constraint->setDamping(i, 1);
		constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
	}
	// get rest length
	restLength = btVector3(constraint->getEquilibriumPoint(0),
		constraint->getEquilibriumPoint(1),
		constraint->getEquilibriumPoint(2)).length();

	// calculate spring constants
	//btScalar k_eq = E_skin * restLength;
	//btScalar k_this = k_eq / 2;
	//btScalar damping_this = getCriticalDampingRatio(m1, m2, k_eq) * 2;
	
	btScalar k_eq = 300;
	btScalar k_this = k_eq / 2;
	btScalar damping_this = getCriticalDampingRatio(m1, m2, k_eq) * 2;

	std::cout << damping_this << std::endl;

	for (int i = 0; i < 3; i++) {
		constraint->setStiffness(i, k_this);
		constraint->setDamping(i, damping_this);
	}



	length = restLength;
	restLengthDefault = restLength;
}

// NEED DEBUG
void Spring::update() {
	// update equilibrium point location in frame 1
	// First, get two attachment points location in world reference frame
	TsP = body1->getCenterOfMassTransform();
	TsP *= T1P;
	TsQ = body2->getCenterOfMassTransform();
	TsQ *= T2Q;
	btVector3 p = TsP.getOrigin();
	btVector3 q = TsQ.getOrigin();
	// Second, get the equilibrium point location in world reference frame
	length = (q - p).length();
	eq = p + (q - p)*restLength / length;
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
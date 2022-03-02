#include "my_pch.h"
#include "Spring.h"

#include "Utility.h"

//////////////////////////////////////////////////////////////////////////////////
// Spring ////////////////////////////////////////////////////////////////////////

Spring::Spring(btRigidBody* b2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	: body2(b2), T1P(frameInParent), T2Q(frameInChild), m_k(k), m_damping(damping) {}

Spring::Spring(btRigidBody* b2, btTransform& frameInChild, btScalar k, btScalar damping)
	: body2(b2), T2Q(frameInChild), m_k(k), m_damping(damping) {}

Spring::~Spring() {
	delete m_constraint;
}


void Spring::debugDraw(btDiscreteDynamicsWorld* m_dynamicsWorld, btVector3 clr) {
	m_dynamicsWorld->getDebugDrawer()->drawLine(TsP.getOrigin(), m_eq, clr);
}

btScalar Spring::getRestLength() const {
	return m_restLength;
}

void Spring::setRestLength(const btScalar ratio) {
	m_restLength = ratio * m_restLengthDefault;
}

btScalar Spring::getLength() const {
	return m_length;
}

btGeneric6DofSpringConstraint* Spring::getConstraint() const {
	return m_constraint;
}

//////////////////////////////////////////////////////////////////////////////////
// SpringBetween /////////////////////////////////////////////////////////////////
SpringBetween::SpringBetween(btRigidBody* b1, btRigidBody* b2,
	btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	: Spring(b2, frameInParent, frameInChild, k, damping)
	, body1(b1) {
	m_constraint = new btGeneric6DofSpringConstraint(*body1, *body2, T1P, T2Q, true);
	init();
};

//SpringBetween::~SpringBetween() {
//	delete m_constraint;
//}


void SpringBetween::init() {
	m_constraint->setLinearLowerLimit(btVector3(1, 1, 1));	// need to set lower > higher to free the dofs
	m_constraint->setLinearUpperLimit(btVector3(0, 0, 0));
	for (int i = 0; i < 3; i++) {
		m_constraint->enableSpring(i, true);
		m_constraint->setStiffness(i, m_k);
		m_constraint->setDamping(i, m_damping);	// guess: damping [0, 1] like restitution coefficient?
		m_constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
	}
	m_restLength = btVector3(m_constraint->getEquilibriumPoint(0),
		m_constraint->getEquilibriumPoint(1),
		m_constraint->getEquilibriumPoint(2)).length();
	m_length = m_restLength;
	m_restLengthDefault = m_restLength;
}

// NEED DEBUG
void SpringBetween::update() {
	// update equilibrium point location in frame 1
	// First, get two attachment points location in world reference frame
	TsP = body1->getCenterOfMassTransform();
	TsP *= T1P;
	TsQ = body2->getCenterOfMassTransform();
	TsQ *= T2Q;
	btVector3 p = TsP.getOrigin();
	btVector3 q = TsQ.getOrigin();
	// Second, get the equilibrium point location in world reference frame
	m_length = (q - p).length();
	m_eq = p + (q - p)*m_restLength / m_length;
	// Third, get the equilibruim point location in body1 reference frame
	btVector3 eq_in_p = TsP.inverse()*m_eq;

	for (int i = 0; i < 3; i++) {
		m_constraint->setEquilibriumPoint(i, eq_in_p[i]);
	}

}



//////////////////////////////////////////////////////////////////////////////////
// SpringAnchor //////////////////////////////////////////////////////////////////
SpringAnchor::SpringAnchor(btRigidBody* b2, btTransform& frameInChild, btScalar k, btScalar damping) 
	: Spring(b2, frameInChild, k, damping) {

	m_constraint = new btGeneric6DofSpringConstraint(*body2, T2Q, true);
	init();
};

//SpringAnchor::~SpringAnchor() {
//	delete m_constraint;
//}

void SpringAnchor::init() {
	m_constraint->setLinearLowerLimit(btVector3(m_k, m_k, m_k));	// need to set lower > higher to free the dofs
															// if k = 0, lock the linear movement (switch to hard anchoring mode)
	m_constraint->setLinearUpperLimit(btVector3(0, 0, 0));
	m_constraint->setAngularLowerLimit(btVector3(1, 1, 1));	// need to set lower > higher to free the dofs
	m_constraint->setAngularUpperLimit(btVector3(0, 0, 0));
	for (int i = 0; i < 3; i++) {
		m_constraint->enableSpring(i, true);
		m_constraint->setStiffness(i, m_k);
		m_constraint->setDamping(i, m_damping);	// guess: damping [0, 1] like restitution coefficient?
		m_constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
	}
	for (int i = 3; i < 6; i++) {
		m_constraint->enableSpring(i, true);
		m_constraint->setStiffness(i, m_k / 10);
		m_constraint->setDamping(i, m_damping);	// guess: damping [0, 1] like restitution coefficient?
		m_constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
	}
	m_restLength = btVector3(m_constraint->getEquilibriumPoint(0),
		m_constraint->getEquilibriumPoint(1),
		m_constraint->getEquilibriumPoint(2)).length();

	m_length = m_restLength;
	m_restLengthDefault = m_restLength;
}

void SpringAnchor::update() {
	// to implement
}



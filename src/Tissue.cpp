#include "my_pch.h"
#include "Tissue.h"

#include "Utility.h"

//////////////////////////////////////////////////////////////////////////////////
// Tissue ////////////////////////////////////////////////////////////////////////

Tissue::Tissue(btRigidBody* b2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	: body2(b2), T1P(frameInParent), T2Q(frameInChild), m_k(k), m_damping(damping) {}

Tissue::Tissue(btRigidBody* b2, btTransform& frameInChild, btScalar k, btScalar damping)
	: body2(b2), T2Q(frameInChild), m_k(k), m_damping(damping) {}

Tissue::~Tissue() {
	delete m_constraint;
}

void Tissue::debugDraw(btDiscreteDynamicsWorld* m_dynamicsWorld, btVector3 clr) {
	m_dynamicsWorld->getDebugDrawer()->drawLine(TsP.getOrigin(), m_eq, clr);
}

btScalar Tissue::getRestLength() const {
	return m_restLength;
}

void Tissue::setRestLength(const btScalar ratio) {
	m_restLength = ratio * m_restLengthDefault;
}

btScalar Tissue::getLength() const {
	return m_length;
}

btGeneric6DofSpringConstraint* Tissue::getConstraint() const {
	return m_constraint;
}

//////////////////////////////////////////////////////////////////////////////////
// TissueBetween /////////////////////////////////////////////////////////////////
TissueBetween::TissueBetween(btRigidBody* b1, btRigidBody* b2,
	btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	: Tissue(b2, frameInParent, frameInChild, k, damping)
	, body1(b1) {

	m_constraint = new btGeneric6DofSpringConstraint(*body1, *body2, T1P, T2Q, true);
	init();
};

//TissueBetween::~TissueBetween() {
//	delete m_constraint;
//}


void TissueBetween::init() {
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
void TissueBetween::update() {
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
// TissueAnchor //////////////////////////////////////////////////////////////////
TissueAnchor::TissueAnchor(btRigidBody* b2, btTransform& frameInChild, btScalar k, btScalar damping) 
	: Tissue(b2, frameInChild, k, damping) {

	m_constraint = new btGeneric6DofSpringConstraint(*body2, T2Q, true);
	init();
};

//TissueAnchor::~TissueAnchor() {
//	delete m_constraint;
//}

void TissueAnchor::init() {
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

void TissueAnchor::update() {
	// to implement
}



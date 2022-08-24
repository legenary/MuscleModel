#include "my_pch.h"
#include "Tissue.h"

#include "Utility.h"

Tissue::Tissue(Simulation* sim, btRigidBody* rbA, btRigidBody* rbB,
	btTransform& frameInA, btTransform& frameInB, 
	btScalar k, btScalar zeta)
	: m_sim(sim), m_k(k), m_type(myTissueType::between), m_rbA(rbA), m_rbB(rbB) {

	m_damping = 2 * sqrt(m_rbB->getMass() * m_k) * zeta;
	//m_constraint = new btGeneric6DofSpringConstraint(*rbA, *rbB, frameInA, frameInB, true);
	m_constraint = new btGeneric6DofSpring2Constraint(*rbA, *rbB, frameInA, frameInB);	// btGeneric6DofSpring2Constraint is preferred for engineering solution
	init();
};

Tissue::Tissue(Simulation* sim, btRigidBody* rbB, btTransform& frameInB,
	btScalar k, btScalar zeta)
	: m_sim(sim), m_k(k), m_type(myTissueType::anchor), m_rbB(rbB) {

	m_damping = 2 * sqrt(m_rbB->getMass() * m_k) * zeta;
	//m_constraint = new btGeneric6DofSpringConstraint(*rbB, frameInB, true);
	m_constraint = new btGeneric6DofSpring2Constraint(*rbB, frameInB);	// btGeneric6DofSpring2Constraint is preferred for engineering solution
	init();
};

Tissue::~Tissue() {
	delete m_constraint;
	m_constraint = nullptr;
	m_rbA = nullptr;
	m_rbB = nullptr;
}

void Tissue::init() {
	m_constraint->setLinearLowerLimit(btVector3(1, 1, 1));	// need to set lower > higher to free the dofs
	m_constraint->setLinearUpperLimit(btVector3(0, 0, 0));
	m_constraint->setAngularLowerLimit(btVector3(1, 1, 1));	
	m_constraint->setAngularUpperLimit(btVector3(0, 0, 0));
	// index 0, 1, 2: translational dimensions
	for (int i = 0; i < 3; i++) {
		m_constraint->enableSpring(i, true);
		m_constraint->setStiffness(i, m_k);
		// viscous damping: m_damping = zeta * 2 * sqrt(m * k)
		m_constraint->setDamping(i, m_damping);
		// set m_equilibriumPoint[index] = distance from A to B
		m_constraint->setEquilibriumPoint(i);
	}

	// This function only works for btGeneric6DofSpringConstraint
	//m_restLength = btVector3(
	//	m_constraint->getEquilibriumPoint(0),
	//	m_constraint->getEquilibriumPoint(1),
	//	m_constraint->getEquilibriumPoint(2)
	//).length();

	btVector3 p = m_constraint->getCalculatedTransformA().getOrigin();
	btVector3 q = m_constraint->getCalculatedTransformB().getOrigin();
	m_restLength = (p - q).length();

	m_length = m_restLength;
	m_restLengthDefault = m_restLength;

	if (m_type == myTissueType::anchor) {
		// the effect torsional damping is to make the system more prune to stablility
		// TODO: remove the damping effect from torsional anchor, to the follice body itself
		for (int i = 3; i < 6; i++) {
			m_constraint->enableSpring(i, true);
			m_constraint->setStiffness(i, m_k / 10);
			m_constraint->setDamping(i, m_damping);
			m_constraint->setEquilibriumPoint(i);   // rest length in three dimension in body1 frame, needs update in stepSimulation
		}
	}

}

// NEED DEBUG
void Tissue::update() {
	// for Tissue: eq points are used to calculate the force direction
	// as well as to update forces using Hooke's Law

	m_constraint->calculateTransforms();
	TsP = m_constraint->getCalculatedTransformA();
	TsQ = m_constraint->getCalculatedTransformB();

	if (m_type == myTissueType::between) {
		// update equilibrium point location in rbA frame
		// First, get two attachment points location in world reference frame

		btVector3 p = TsP.getOrigin();
		btVector3 q = TsQ.getOrigin();
		// Second, get the equilibrium point location in world reference frame
		m_length = (q - p).length();
		m_eq = p + (q - p)*m_restLength / m_length;
		// Third, get the equilibruim point location in body1 reference frame
		btVector3 eq_in_p = TsP.inverse()*m_eq;

		// set equilibruim point
		for (int i = 0; i < 3; i++) {
			m_constraint->setEquilibriumPoint(i, eq_in_p[i]);
		}
	}
	else if (m_type == myTissueType::anchor) {
		// equilibrium point is relative to rbA, in this case it's [0, 0, 0]
		// no matter how rbB moves, eq will not move, and will always be [0, 0, 0]
		// therefore there is no need to do anything in anchor.update()
		m_eq = TsP.getOrigin();
	}

}


void Tissue::debugDraw(btVector3 clr, bool dynamic) {
	if (dynamic)
		getWorld()->getDebugDrawer()->drawLine(TsQ.getOrigin(), m_eq, clr);
	else
		getWorld()->getDebugDrawer()->drawLine(TsP.getOrigin(), m_eq, clr);
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

btGeneric6DofSpring2Constraint* Tissue::getConstraint() const {
	return m_constraint;
}




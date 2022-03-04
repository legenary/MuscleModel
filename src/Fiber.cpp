#include "my_pch.h"
#include "Fiber.h"

#include "myGeneric6DofMuscleConstraint.h"
#include "Utility.h"

Fiber::Fiber(btRigidBody* rbA, btRigidBody* rbB,
	btTransform& frameInA, btTransform& frameInB,
	btScalar k, btScalar damping) : m_k(k), m_damping(damping) {

	m_constraint = new myGeneric6DofMuscleConstraint(*rbA, *rbB, frameInA, frameInB, true);
	init();
};

Fiber::Fiber(btRigidBody* rbB, btTransform& frameInB,
	btScalar k, btScalar damping) : m_k(k), m_damping(damping){

	m_constraint = new myGeneric6DofMuscleConstraint(*rbB, frameInB, true);
	init();
};

Fiber::~Fiber() {
	delete m_constraint;
}

void Fiber::init() {
	m_constraint->setLinearLowerLimit(btVector3(1, 1, 1));	// need to set lower > higher to free the dofs
	m_constraint->setLinearUpperLimit(btVector3(0, 0, 0));
	m_constraint->setAngularLowerLimit(btVector3(1, 1, 1));
	m_constraint->setAngularUpperLimit(btVector3(0, 0, 0));
	// index 0, 1, 2: translational dimensions
	for (int i = 0; i < 3; i++) {
		m_constraint->enableSpring(i, true);
		m_constraint->setStiffness(i, m_k);
		// not sure how damping coefficient is defined
		// guess: damping [0, 1] like restitution coefficient?
		m_constraint->setDamping(i, m_damping);
		// set m_equilibriumPoint[index] = distance from A to B
		m_constraint->setEquilibriumPoint(i);
	}

	m_restLength = btVector3(
		m_constraint->getEquilibriumPoint(0),
		m_constraint->getEquilibriumPoint(1),
		m_constraint->getEquilibriumPoint(2)
	).length();

	m_length = m_restLength;
	m_restLengthDefault = m_restLength;

}

// NEED DEBUG
void Fiber::update() {
	m_constraint->calculateTransforms();
	TsP = m_constraint->getCalculatedTransformA();
	TsQ = m_constraint->getCalculatedTransformB();


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


void Fiber::debugDraw(btDiscreteDynamicsWorld* world, btVector3 clr, bool dynamic) {
	if (dynamic)
		world->getDebugDrawer()->drawLine(TsQ.getOrigin(), m_eq, clr);
	else
		world->getDebugDrawer()->drawLine(TsP.getOrigin(), m_eq, clr);
}

btScalar Fiber::getRestLength() const {
	return m_restLength;
}

void Fiber::setRestLength(const btScalar ratio) {
	m_restLength = ratio * m_restLengthDefault;
}

btScalar Fiber::getLength() const {
	return m_length;
}

btGeneric6DofSpringConstraint* Fiber::getConstraint() const {
	return m_constraint;
}




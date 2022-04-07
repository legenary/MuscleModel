#include "my_pch.h"
#include "Fiber.h"

#include "myGeneric6DofMuscleConstraint.h"
#include "Utility.h"

// constructor
Fiber::Fiber(Simulation* sim, btRigidBody* rbA, btRigidBody* rbB, 
	btTransform& frameInA, btTransform& frameInB, btScalar k, btScalar damping, int idx) 
	: m_sim(sim)
	, f0(100)
	, m_k(k)
	, m_damping(damping)
	, m_idx(idx)
	, m_velocity(0)
	, m_activation(0) {

	m_constraint = new myGeneric6DofMuscleConstraint(*rbA, *rbB, frameInA, frameInB, true);
	init();
};

Fiber::Fiber(Simulation* sim, btRigidBody* rbB, btTransform& frameInB,
	btScalar k, btScalar damping) 
	: m_sim(sim), m_k(k), m_damping(damping){

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
		// stiffness is practically useless because we're directly updating forces?
		m_constraint->setStiffness(i, m_k);
		// not sure how damping coefficient is defined
		// guess: damping [0, 1] like restitution coefficient?
		m_constraint->setDamping(i, m_damping);
		// set m_equilibriumPoint[index] = distance from A to B
		m_constraint->setEquilibriumPoint(i);
	}

	m_length = btVector3(
		m_constraint->getEquilibriumPoint(0),
		m_constraint->getEquilibriumPoint(1),
		m_constraint->getEquilibriumPoint(2)
	).length();

	m_restLength= m_length;
	m_restLengthPassive = m_length;

}

// NEED DEBUG
void Fiber::update() {
	// for Fiber: eq points are used to calculate the force direction
	// but no longer used to update forces using Hooke's Law
	// the force is calcualted from Hill-type muscle model

	m_constraint->calculateTransforms();
	TsP = m_constraint->getCalculatedTransformA();
	TsQ = m_constraint->getCalculatedTransformB();
	// if passive: passive restlength stays unchanged
	// update equilibrium point location in rbA frame
		// First, get two attachment points location in world reference frame
	btVector3 p = TsP.getOrigin();
	btVector3 q = TsQ.getOrigin();
		// Second, get the equilibrium point location in world reference frame
	btScalar vLength = (q - p).length()/m_length - 1.0; // muscle velocity
	m_length = (q - p).length();
	m_eq = p + (q - p)*m_restLength / m_length;
		// Third, get the equilibruim point location in body1 reference frame
	btVector3 eq_in_p = TsP.inverse()*m_eq;
		// set equilibruim point
	for (int i = 0; i < 3; i++) 
		m_constraint->setEquilibriumPoint(i, eq_in_p[i]);

	// force along direction of equilibrium points
	btVector3 dir = (m_constraint->getCalculatedLinearDiff() - eq_in_p);
	dir = (dir.length() > 0.02) ? dir / dir.length() : btVector3(0, 0, 0);
	// force = (fPE + a*fL*fV)
	btVector3 force = f0 * (
					interp1(fPE[0], fPE[1], m_length / m_restLength) +
					m_activation * interp1(fL[0], fL[1], m_length / m_restLength)
								 * interp1(fV[0], fV[1], vLength)
					) * dir;
	updateNetForce(force);


}


void Fiber::debugDraw(btVector3 clr, bool dynamic) {
	if (dynamic)
		getWorld()->getDebugDrawer()->drawLine(TsQ.getOrigin(), m_eq, clr);
	else
		getWorld()->getDebugDrawer()->drawLine(TsP.getOrigin(), m_eq, clr);
}

btScalar Fiber::getRestLength() const {
	return m_restLength;
}

void Fiber::contractTo(btScalar ratio) {
	if (ratio < 0.6 || ratio > 1.0) {
		std::cerr << "Invalid muscle contraction ratio. Should be between 0.6 and 1.0.\n";
	}
	m_activation = (1.0 - ratio) / 0.4;
	setRestLength(ratio);

	/*static int cc = 0;
	cc++;
	if (cc == 1) {
		std::cout << m_length / m_restLength << std::endl;
		std::cout << interp1(fPE[0], fPE[1], m_length / m_restLength) << std::endl;
		std::cout << interp1(fL[0], fL[1], m_length / m_restLength) << std::endl;
	}*/
	
}

void Fiber::setRestLength(const btScalar ratio) {
	m_restLength = ratio * m_restLengthPassive;
}

btScalar Fiber::getLength() const {
	return m_length;
}

btGeneric6DofSpringConstraint* Fiber::getConstraint() const {
	return m_constraint;
}

// pass calculate linear force to the constraint
void Fiber::updateNetForce(btVector3 force) {
	m_constraint->updateForce(force);
}
#include "my_pch.h"
#include "Fiber.h"

#include "Parameter.h"
#include "Utility.h"
#include "Simulation.h"

// constructor
Fiber::Fiber(Simulation* sim, btRigidBody* rbA, btRigidBody* rbB, 
	btTransform& frameInA, btTransform& frameInB, btScalar f0, int idx) 
	: m_sim(sim)
	, m_f0(f0)
	, m_idx(idx)
	, m_velocity(0)
	, m_excitation(0)
	, m_activation(0)
	, m_Hamiltonian(0) 
	, m_force_magnitude(0) {
	m_constraint = new myGeneric6DofMuscleConstraint(*rbA, *rbB, frameInA, frameInB, true);
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
		m_constraint->setStiffness(i, 0 /*spring stiffness is not used*/);
		m_constraint->setDamping(i, 1 /*no dmaping, and not used either*/);
		// set m_equilibriumPoint[index] = distance from A to B
		m_constraint->setEquilibriumPoint(i);
	}

	//m_length = btVector3(
	//	m_constraint->getEquilibriumPoint(0),
	//	m_constraint->getEquilibriumPoint(1),
	//	m_constraint->getEquilibriumPoint(2)
	//).length();
	btVector3 p = m_constraint->getCalculatedTransformA().getOrigin();
	btVector3 q = m_constraint->getCalculatedTransformB().getOrigin();
	m_restLength = (p - q).length();

	m_length = m_restLength;
	m_prev_length = m_length;
	m_restLengthNoAvtivation = m_length;
	m_prev_force = btVector3(0., 0., 0.);
	m_hill_model_components = btVector3(0., 0., 0.);

	m_constraint->enableFeedback(true);

	m_activation_tau = m_sim->getParameter()->muslce_activation_tau;
}

//*This is run per fiber query (60Hz)
void Fiber::update() {
	// update activation based on neural excitation
	btScalar da = (m_excitation - m_activation) / m_activation_tau;
	m_activation += da * m_sim->getParameter()->m_time_step;

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
	btScalar vLength = (q - p).length()/m_length - 1.0; // muscle velocity? double-check
														// muscle velocity needs to be normalized to v_max
														// which is a parameter decided by the model
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
	// split it up for debugging
		// Method 1: use current frame length
	//btScalar this_fPE = interp1(fPE[0], fPE[1], m_length / m_restLength);
	//btScalar this_fL = interp1(fL[0], fL[1], m_length / m_restLength);
	//btScalar this_fV = interp1(fV[0], fV[1], vLength);
		// Method 2: use midpoint of current frame length and future frame length (projected from prev)
	btScalar future_length = 2 * m_length - m_prev_length;
	btScalar mid_length = (m_length + future_length) / 2;
	btScalar this_fPE = interp1(fPE[0], fPE[1], mid_length / m_restLength);
	btScalar this_fL = interp1(fL[0], fL[1], mid_length / m_restLength);
	btScalar this_fV = interp1(fV[0], fV[1], vLength);

	m_hill_model_components = { this_fPE, this_fL, this_fV };
	m_force = m_f0
			* (this_fPE + m_activation * this_fL * this_fV)
			* dir;
	m_force_magnitude = m_force.length();
	m_constraint->updateForce(m_force);

	//// debug, output the 12th intrinsic muscle info (full array)
	////					 2nd  intrinsic muscle info (reduced array)
	//if (m_idx == 2) {
	//	S_dumpster::Get().fiber_info[0].push_back(m_restLength);
	//	S_dumpster::Get().fiber_info[1].push_back(m_length);
	//	S_dumpster::Get().fiber_info[2].push_back(m_force.length());		// instructed force
	//	S_dumpster::Get().fiber_info[3].push_back(this_fPE);
	//	S_dumpster::Get().fiber_info[4].push_back(this_fL);
	//	S_dumpster::Get().fiber_info[5].push_back(this_fV);

	//	btScalar internalImpusle = m_constraint->getAppliedImpulse();	// actual impulse applied for each internal step
	//	btScalar fixedTimeStep = getWorld()->getSolverInfo().m_timeStep;
	//	S_dumpster::Get().fiber_info[6].push_back(internalImpusle / fixedTimeStep);	// actual force

	//	//btScalar b = getWorld()->getDispatchInfo().m_stepCount;
	//	//btJointFeedback* jfb = m_constraint->getJointFeedback();
	//	//btVector3 forceA = jfb->m_appliedForceBodyA;
	//	//S_dumpster::Get()->fiber_info[6].push_back(forceA.length());
	//}
	
	// update hamiltonian as the potential energy, the potential enery is calcualted as the work done by the force
	// the potential enery is cleared every contraction and relaxation
	if (m_sim->flagMuscleContractionStateChange) {
		m_Hamiltonian = 0;
	}
	m_Hamiltonian += (m_prev_force.length() + m_force.length()) * 0.5 * btFabs(m_length - m_prev_length);

	m_prev_length = m_length;
	m_prev_force = m_force;
}


void Fiber::debugDraw(btVector3 clr, bool dynamic) {
	if (dynamic)
		getWorld()->getDebugDrawer()->drawLine(TsQ.getOrigin(), m_eq, clr);
	else
		getWorld()->getDebugDrawer()->drawLine(TsP.getOrigin(), m_eq, clr);
}

void Fiber::contractTo(btScalar ratio) {
	if (ratio < 0.5 || ratio > 1.0) {
		std::cerr << "Invalid muscle contraction ratio. Should be between 0.6 and 1.0.\n";
	}
	m_excitation = ratio2excitation(ratio); // old code
	setRestLengthRatio(ratio);



	// debug
	/*static int cc = 0;
	cc++;
	if (cc == 1) {
		std::cout << m_length / m_restLength << std::endl;
		std::cout << interp1(fPE[0], fPE[1], m_length / m_restLength) << std::endl;
		std::cout << interp1(fL[0], fL[1], m_length / m_restLength) << std::endl;
	}*/
}

btScalar Fiber::ratio2excitation(btScalar ratio) {
	// this is for old method where muscle activation is associated with ratio
	// and because the (instructed) ratio usually changes suddenly
	// the muscle activation will also change suddenly
	if (ratio == 1) { return 0; }
	return (1.0 - ratio) / 0.5;
}

void Fiber::setRestLengthRatio(const btScalar ratio) {
	m_restLength = ratio * m_restLengthNoAvtivation;
}


const btScalar& Fiber::getRestLength() const {
	return m_restLength;
}
const btScalar& Fiber::getLength() const {
	return m_length;
}
const btScalar& Fiber::getForce() const {
	return m_force_magnitude;
}
const btVector3& Fiber::getForceHillModelComps() const {
	return m_hill_model_components;
}
const btScalar& Fiber::getExcitation() const {
	return m_excitation;
}
const btScalar& Fiber::getActivation() const {
	return m_activation;
}

myGeneric6DofMuscleConstraint* Fiber::getConstraint() const {
	return m_constraint;
}

btDynamicsWorld* Fiber::getWorld() {
	return m_sim->getDynamicsWorld();
}


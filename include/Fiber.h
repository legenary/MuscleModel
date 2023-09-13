// Fiber is different from Tissue that it has a different type of constraint
// Fiber also has more functionality such as contract and will replace Muscle::contract in the future release

#ifndef FIBER_H
#define FIBER_H

#include "myGeneric6DofMuscleConstraint.h"


class Simulation;

class Fiber {
protected:
	Simulation* m_sim;

	btScalar m_f0;
	int m_idx;

	myGeneric6DofMuscleConstraint* m_constraint;
	btTransform TsP;
	btTransform TsQ;
	btVector3 m_eq;	// equilibrium point that is restLength from body 1 frame
					// equilibrium point is represented in world frame

	btScalar m_restLength;	
	btScalar m_restLengthNoAvtivation;
	btScalar m_length, m_prev_length;		// fiber length
	btScalar m_velocity;					// fiber lengthening/shortening velocity
	btScalar m_excitation;					// neural excitation, between 0 and 1
	btScalar m_activation;					// muscle activation level
	btScalar m_activation_tau_a;			// activation time constant
	btScalar m_activation_tau_d;			// deactivation time constant

	btVector3 m_force, m_prev_force;
	btVector3 m_hill_model_components;
	btScalar m_force_magnitude;

	btScalar m_Hamiltonian;

	btScalar calculateTau();

public:
	Fiber(Simulation* sim, btRigidBody* rbA, btRigidBody* rbB,
		btTransform& frameInA, btTransform& frameInB,
		btScalar f0, int idx = 0);
	// disable copy constructor (override if needed in the future)
	Fiber(const Fiber&) = delete;
	Fiber& operator=(Fiber const&) = delete;
	virtual ~Fiber();

	// override new and delete operator to make sure object is aligned 16 on the heap
	void* operator new(size_t i) { return _mm_malloc(i, 16); }
	void operator delete(void* p) { _mm_free(p); }

	
	void init();
	void preUpdate();
	void postUpdate();
	void debugDraw(btVector3 clr = btVector3(0., 0., 0.), bool dynamic = false);

	myGeneric6DofMuscleConstraint* getConstraint() const;
	const btScalar& getRestLength() const;
	const btScalar& getLength() const;
	const btScalar& getForce() const;
	const btVector3& getForceHillModelComps() const;
	const btScalar& getExcitation() const;
	const btScalar& getActivation() const;
	btDynamicsWorld* getWorld();

	void contractTo(btScalar ratio);
	btScalar getHamiltonian() const { return m_Hamiltonian; }

	void setTorsionalSpring(btVector3 stiffnesses, btVector3 dampings);

	
private:
	void setRestLengthRatio(const btScalar ratio);
	std::vector<std::vector<btScalar>> fPE{
		{1.0,	1.1,	1.2,	1.3,	1.4,	1.5,	1.6,	1.7,	1.8,	1.9,	2.0},
		{0.0,	0.01,	0.04,	0.11,	0.26,	0.45,	0.7,	1.0,	1.3,	1.6,	2.0}
	};
	std::vector<std::vector<btScalar>> fL{
		{0.5,	0.7, 0.95,	1.05,	1.1,	1.8},
		{0.0,	0.75, 1.0,	1.0,	0.96,	0.0}
	};
	std::vector<std::vector<btScalar>> fV{
		{-1.0,	-0.8,	-0.6,	-0.5,	-0.4,	-0.3,	-0.2,	-0.1,	-0.05,	0.0,	0.05,	0.1,	0.2,	1.0},
		{0.0,	0.04,	0.11,	0.16,	0.24,	0.34,	0.46,	0.64,	0.8,	1.0,	1.16,	1.23,	1.28,	1.4}
	};
	btScalar ratio2excitation(btScalar ratio);
	


};







#endif
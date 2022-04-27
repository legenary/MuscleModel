// Fiber is different from Tissue that it has a different type of constraint
// Fiber also has more functionality such as contract and will replace Muscle::contract in the future release

#ifndef FIBER_H
#define FIBER_H

#include "Simulation.h"

class myGeneric6DofMuscleConstraint;

class Fiber {
protected:
	Simulation* m_sim;

	btScalar f0;
	btScalar m_k;
	btScalar m_damping;
	int m_idx;

	myGeneric6DofMuscleConstraint* m_constraint;
	btTransform TsP;
	btTransform TsQ;
	btVector3 m_eq;	// equilibrium point that is restLength from body 1 frame
					// equilibrium point is represented in world frame

	btScalar m_restLength;	
	btScalar m_restLengthPassive;
	btScalar m_length;				// fiber length
	btScalar m_velocity;			// fiber lengthening/shortening velocity
	btScalar m_activation;			// activation level

public:
	//Tissue(btScalar k, btScalar damping);
	Fiber(Simulation* sim, btRigidBody* rbA, btRigidBody* rbB,
		btTransform& frameInA, btTransform& frameInB,
		btScalar k, btScalar damping, int idx = 0);
	Fiber(Simulation* sim, btRigidBody* rbB, btTransform& frameInB,
		btScalar k, btScalar damping);
	// disable copy constructor (override if needed in the future)
	Fiber(const Fiber&) = delete;
	Fiber& operator=(Fiber const&) = delete;
	virtual ~Fiber();

	// override new and delete operator to make sure object is aligned 16 on the heap
	void* operator new(size_t i) { return _mm_malloc(i, 16); }
	void operator delete(void* p) { _mm_free(p); }

	
	void init();
	void update();
	void debugDraw(btVector3 clr = btVector3(0., 0., 0.), bool dynamic = false);

	btGeneric6DofSpringConstraint* getConstraint() const;
	btScalar getRestLength() const;
	btScalar getLength() const;
	inline btDynamicsWorld* getWorld() { return m_sim->getDynamicsWorld(); }

	void contractTo(btScalar ratio);


	
private:
	void setRestLengthRatio(const btScalar ratio);
	std::vector<std::vector<btScalar>> fPE{
		{1.0,	1.1,	1.2,	1.3,	1.4,	1.5,	1.6,	1.7,	1.8,	1.9,	2.0},
		{0.0,	0.01,	0.04,	0.11,	0.26,	0.45,	0.7,	1.0,	1.3,	1.6,	2.0}
	};
	std::vector<std::vector<btScalar>> fL{
		{0.5,	0.95,	1.05,	1.1,	1.8},
		{0.0,	1.0,	1.0,	0.96,	0.0}
	};
	std::vector<std::vector<btScalar>> fV{
		{-1.0,	-0.8,	-0.6,	-0.5,	-0.4,	-0.3,	-0.2,	-0.1,	-0.05,	0.0,	0.05,	0.1,	0.2,	1.0},
		{0.0,	0.04,	0.11,	0.16,	0.24,	0.34,	0.46,	0.64,	0.8,	1.0,	1.16,	1.23,	1.28,	1.4}
	};
	btScalar ratio2activation(btScalar ratio);
	


};







#endif
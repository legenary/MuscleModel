#ifndef TISSUE_H
#define TISSUE_H

#include "Simulation.h"

enum class myTissueType {
	between = 0,
	anchor = 1
};

class Tissue {
protected:
	Simulation* m_sim;

	btScalar m_k;
	btScalar m_damping;
	myTissueType m_type;

	btGeneric6DofSpring2Constraint* m_constraint;
	btTransform TsP;
	btTransform TsQ;
	btVector3 m_eq;	// equilibrium point that is restLength from body 1 frame
					// equilibrium point is represented in world frame

	btScalar m_restLength;
	btScalar m_restLengthDefault;
	btScalar m_length;

	btRigidBody* m_rbA, * m_rbB;
	btScalar m_Hamiltonian;

public:
	//Tissue(btScalar k, btScalar damping);
	Tissue(Simulation* sim, btRigidBody* rbA, btRigidBody* rbB,
		btTransform& frameInA, btTransform& frameInB,
		btScalar k, btScalar zeta);
	Tissue(Simulation* sim, btRigidBody* rbB, btTransform& frameInB,
		btScalar k, btScalar zeta);
	// disable copy constructor (override if needed in the future)
	Tissue(const Tissue&) = delete;
	Tissue& operator=(Tissue const&) = delete;
	virtual ~Tissue();

	// override new and delete operator to make sure object is aligned 16 on the heap
	void* operator new(size_t i) { return _mm_malloc(i, 16); }
	void operator delete(void* p) { _mm_free(p); }

	void update();
	void init();

	btGeneric6DofSpring2Constraint* getConstraint() const;
	btScalar getRestLength() const;
	void setRestLength(const btScalar ratio);
	btScalar getLength() const;

	void debugDraw(const btVector3 clr = btVector3(0., 0., 0.), bool dynamic = false);
	inline btDynamicsWorld* getWorld() {
		return m_sim->getDynamicsWorld();
	}
	btScalar getHamiltonian() const { return m_Hamiltonian; }
	
};

#endif
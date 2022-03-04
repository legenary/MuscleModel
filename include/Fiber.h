// Fiber is different from Tissue that it has a different type of constraint
// Fiber also has more functionality such as contract and will replace Muscle::contract in the future release

#ifndef FIBER_H
#define FIBER_H

class myGeneric6DofMuscleConstraint;

class Fiber {
protected:
	btScalar m_k;
	btScalar m_damping;

	myGeneric6DofMuscleConstraint* m_constraint;
	btTransform TsP;
	btTransform TsQ;
	btVector3 m_eq;	// equilibrium point that is restLength from body 1 frame
					// equilibrium point is represented in world frame

	btScalar m_restLength;
	btScalar m_restLengthDefault;
	btScalar m_length;

public:
	//Tissue(btScalar k, btScalar damping);
	Fiber(btRigidBody* rbA, btRigidBody* rbB,
		btTransform& frameInA, btTransform& frameInB,
		btScalar k, btScalar damping);
	Fiber(btRigidBody* rbB, btTransform& frameInB,
		btScalar k, btScalar damping);
	// disable copy constructor (override if needed in the future)
	Fiber(const Fiber&) = delete;
	Fiber& operator=(Fiber const&) = delete;
	virtual ~Fiber();

	// override new and delete operator to make sure object is aligned 16 on the heap
	void* operator new(size_t i) { return _mm_malloc(i, 16); }
	void operator delete(void* p) { _mm_free(p); }

	void update();
	void init();

	btGeneric6DofSpringConstraint* getConstraint() const;
	btScalar getRestLength() const;
	void setRestLength(const btScalar ratio);
	btScalar getLength() const;

	void debugDraw(btDiscreteDynamicsWorld* world, btVector3 clr = btVector3(0., 0., 0.), bool dynamic = false);
};








#endif
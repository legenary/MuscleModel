#ifndef TISSUE_H
#define TISSUE_H

enum type {
	between = 0,
	anchor = 1
};

class Tissue {
protected:
	btScalar m_k;
	btScalar m_damping;
	type m_type;

	btGeneric6DofSpringConstraint* m_constraint;
	btTransform TsP;
	btTransform TsQ;
	btVector3 m_eq;	// equilibrium point that is restLength from body 1 frame
					// equilibrium point is represented in world frame

	btScalar m_restLength;
	btScalar m_restLengthDefault;
	btScalar m_length;

public:
	//Tissue(btScalar k, btScalar damping);
	Tissue(btRigidBody* rbA, btRigidBody* rbB,
		btTransform& frameInA, btTransform& frameInB,
		btScalar k, btScalar damping);
	Tissue(btRigidBody* rbB, btTransform& frameInB,
		btScalar k, btScalar damping);
	// disable copy constructor (override if needed in the future)
	Tissue(const Tissue&) = delete;
	Tissue& operator=(Tissue const&) = delete;
	virtual ~Tissue();

	// override new and delete operator to make sure object is aligned 16 on the heap
	void* operator new(size_t i) { return _mm_malloc(i, 16); }
	void operator delete(void* p) { _mm_free(p); }

	virtual void update();
	virtual void init();

	btGeneric6DofSpringConstraint* getConstraint() const;
	btScalar getRestLength() const;
	void setRestLength(const btScalar ratio);
	btScalar getLength() const;

	void debugDraw(btDiscreteDynamicsWorld* world, btVector3 clr = btVector3(0., 0., 0.));
};


//// derived class
//class TissueBetween : public Tissue {
//public:
//	TissueBetween(btRigidBody* rbA, btRigidBody* rbB,
//		btTransform& frameInA, btTransform& frameInB, 
//		btScalar k, btScalar damping);
//	virtual ~TissueBetween() override {};
//	
//	virtual void init() override;
//	virtual void update() override;
//}; 
//
//
//class TissueAnchor : public Tissue {
//public:
//	TissueAnchor(btRigidBody* rbB, btTransform& frameInB,
//		btScalar k, btScalar damping);
//	virtual ~TissueAnchor() override {};
//	
//	virtual void init() override;
//	virtual void update() override;
//};

#endif
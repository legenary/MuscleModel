#ifndef TISSUE_H
#define TISSUE_H

// abstract class (derived class in the same file)
class Tissue {
protected:
	
	btRigidBody* body2;	// body2 is this body
	btTransform T1P;
	btTransform T2Q;
	btScalar m_k;
	btScalar m_damping;

	btGeneric6DofSpringConstraint* m_constraint;
	btTransform TsP;
	btTransform TsQ;
	btVector3 m_eq;	// equilibrium point that is restLength from body 1 frame
					// equilibrium point is represented in world frame

	btScalar m_restLength;
	btScalar m_restLengthDefault;
	btScalar m_length;

public:
	Tissue(btRigidBody* b2, btTransform& frameInParent, btTransform& frameInChild, 
		btScalar k, btScalar damping);
	Tissue(btRigidBody* b2, btTransform& frameInChild,
		btScalar k, btScalar damping);
	Tissue(const Tissue&) = delete;
	Tissue& operator=(Tissue const&) = delete;
	virtual ~Tissue();

	void* operator new(size_t i) { return _mm_malloc(i, 16); }
	void operator delete(void* p) { _mm_free(p); }

	virtual void update() = 0;
	virtual void init() = 0;

	btGeneric6DofSpringConstraint* getConstraint() const;
	btScalar getRestLength() const;
	void setRestLength(const btScalar ratio);
	btScalar getLength() const;

	void debugDraw(btDiscreteDynamicsWorld* world, btVector3 clr = btVector3(0., 0., 0.));
};


// derived class
class TissueBetween : public Tissue {
protected:
	btRigidBody* body1;	//body1 is other body
public:
	TissueBetween(btRigidBody* b1, btRigidBody* b2, 
		btTransform& frameInParent, btTransform& frameInChild, 
		btScalar k, btScalar damping);
	virtual ~TissueBetween() override {};
	
	virtual void init() override;
	virtual void update() override;
}; 


class TissueAnchor : public Tissue {
public:
	TissueAnchor(btRigidBody* b2, btTransform& frameInChild, 
		btScalar k, btScalar damping);
	virtual ~TissueAnchor() override {};
	
	virtual void init() override;
	virtual void update() override;
};

#endif
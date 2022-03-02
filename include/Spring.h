#ifndef SPRING_H
#define SPRING_H

// abstract class (derived class in the same file)
class Spring {
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
	Spring(btRigidBody* b2, btTransform& frameInParent, btTransform& frameInChild, 
		btScalar k, btScalar damping);
	Spring(btRigidBody* b2, btTransform& frameInChild, 
		btScalar k, btScalar damping);
	Spring(const Spring&) = delete;
	Spring& operator=(Spring const&) = delete;
	virtual ~Spring();

	virtual void update() = 0;
	virtual void init() = 0;

	btGeneric6DofSpringConstraint* getConstraint() const;
	btScalar getRestLength() const;
	void setRestLength(const btScalar ratio);
	btScalar getLength() const;

	void debugDraw(btDiscreteDynamicsWorld* world, btVector3 clr = btVector3(0., 0., 0.));
};


// derived class
class SpringBetween : public Spring {
protected:
	btRigidBody* body1;	//body1 is other body
public:
	SpringBetween(btRigidBody* b1, btRigidBody* b2, 
		btTransform& frameInParent, btTransform& frameInChild, 
		btScalar k, btScalar damping);
	virtual ~SpringBetween() override {};
	
	virtual void init() override;
	virtual void update() override;
}; 


class SpringAnchor : public Spring {
public:
	SpringAnchor(btRigidBody* b2, btTransform& frameInChild, 
		btScalar k, btScalar damping);
	virtual ~SpringAnchor() override {};
	
	virtual void init() override;
	virtual void update() override;
};

#endif
#ifndef FOLLICLE_H
#define FOLLICLE_H

#include "MystacialPad.h"

class Follicle {
private:
	//btVector3 m_position;
	//btVector3 m_orientation;
	MystacialPad* m_pad;

	btScalar m_mass;
	btScalar m_length;
	int m_index;
	
	btRigidBody* m_body;
	btCollisionShape* m_shape;

public:
	Follicle(MystacialPad* pad, btTransform trans, btScalar radius, btScalar half_height, btScalar mass, int f);
	// disable copy constructor (override if needed in the future)
	Follicle(const Follicle&) = delete;
	Follicle& operator=(Follicle const&) = delete;
	virtual ~Follicle();

	btRigidBody* getBody() const;
	btScalar getLength() const;
	btScalar getMass() const;
	int getIndex() const;
	
	inline btDynamicsWorld* getWorld() {
		return m_pad->getWorld();
	}
	inline btAlignedObjectArray<btCollisionShape*>* getCollisionShapes() {
		return m_pad->getCollisionShapes();
	}


};








#endif
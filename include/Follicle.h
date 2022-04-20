#ifndef FOLLICLE_H
#define FOLLICLE_H

#include "MystacialPad.h"

struct Follicle_info {
	int userIndex;
	Follicle_info() : userIndex(0) {}
	Follicle_info(int idx) : userIndex(idx) {}
};

class Follicle {
private:
	//btVector3 m_position;
	//btVector3 m_orientation;
	MystacialPad* m_pad;

	btScalar m_mass;
	btScalar m_length;
	
	btRigidBody* m_body;
	btCollisionShape* m_shape;
	Follicle_info* m_info;

public:
	Follicle(MystacialPad* pad, btTransform trans, btScalar radius, btScalar half_height, btScalar mass, int f);
	// disable copy constructor (override if needed in the future)
	Follicle(const Follicle&) = delete;
	Follicle& operator=(Follicle const&) = delete;
	virtual ~Follicle();

	btRigidBody* getBody() const;
	btScalar getLength() const;
	btScalar getMass() const;
	void* getUserPointer() const;
	void setUserPointer(void* idxuserPointer);
	inline Follicle_info* getInfo() const { return m_info; }
	
	inline btDynamicsWorld* getWorld() {
		return m_pad->getWorld();
	}
	inline btAlignedObjectArray<btCollisionShape*>* getCollisionShapes() {
		return m_pad->getCollisionShapes();
	}


};








#endif
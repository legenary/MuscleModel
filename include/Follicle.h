#ifndef FOLLICLE_H
#define FOLLICLE_H

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

#include "Utility.h"

class Follicle {
private:
	btVector3 position;
	btVector3 orientation;
	btRigidBody* body;

public:
	Follicle(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, 
		     btTransform trans, btScalar radius, btScalar height, btScalar mass);
	virtual ~Follicle(){}

	btRigidBody* getBody();
	



};








#endif
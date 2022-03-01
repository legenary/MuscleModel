#ifndef FOLLICLE_H
#define FOLLICLE_H

//#include "btBulletDynamicsCommon.h"
//#include "LinearMath/btVector3.h"

#include "Utility.h"

class Follicle {
private:
	btVector3 position;
	btVector3 orientation;
	btRigidBody* body;
	btScalar l;
	btScalar m;
	

public:
	Follicle(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, 
		     btTransform trans, btScalar radius, btScalar half_height, btScalar mass);
	virtual ~Follicle(){}

	btRigidBody* getBody();
	btScalar getLength();
	btScalar getMass();
	



};








#endif
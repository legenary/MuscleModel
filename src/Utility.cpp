#include "Utility.h"

btRigidBody* createDynamicBody(float mass, const btTransform& startTransform, btCollisionShape* shape) {
	// rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic) {
		shape->calculateLocalInertia(mass, localInertia);
	}
	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
	//cInfo.m_restitution = restitution;
	//cInfo.m_friction = friction;
	btRigidBody* body = new btRigidBody(cInfo);
	body->setUserIndex(-1);

	return body;
}

btTransform createTransform(btVector3 origin, btVector3 rotation) {
	// rotation: EuelrZYX rotation orders

	btTransform trans;
	trans = btTransform::getIdentity();
	trans.setOrigin(origin);
	trans.getBasis().setEulerZYX(rotation[0], rotation[1], rotation[2]);
	return trans;
}


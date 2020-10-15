#include "Utility.h"

#define BIT(x) (1<<(x))

enum collisiontypes {
	COL_NOTHING = 0, //Collide with nothing
	COL_FOLLICLE = BIT(0), //Collide with head
	COL_INT_MUS = BIT(1), //Collide with follicles
	COL_EXT_MUS = BIT(2),		// Collide with base
};

static int follicleCollideWith = COL_FOLLICLE;
static int intMusCollideWith = COL_FOLLICLE | COL_EXT_MUS;
static int extMusCollideWith = COL_FOLLICLE | COL_INT_MUS;


btRigidBody* createDynamicBody(float mass, const btTransform& startTransform, btCollisionShape* shape, GUIHelperInterface* m_guiHelper) {
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


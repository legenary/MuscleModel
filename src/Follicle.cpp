#include "Follicle.h"

Follicle::Follicle(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes,
	               btVector3 pos, btVector3 orient, btScalar radius, btScalar height) {

	btTransform trans = createTransform(pos, orient);
	btCollisionShape* shape = new btCylinderShapeX(btVector3(height, radius, radius));
	m_collisionShapes->push_back(shape);
	btRigidBody* this_follicle_body = createDynamicBody(1., trans, shape);
	m_dynamicsWorld->addRigidBody(this_follicle_body, COL_FOLLICLE, follicleCollideWith);
	this_follicle_body->setActivationState(DISABLE_DEACTIVATION);

}




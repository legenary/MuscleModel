#include "Follicle.h"

Follicle::Follicle(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes,
	               btTransform trans, btScalar radius, btScalar height, btScalar mass) {

	btCollisionShape* shape = new btCylinderShapeX(btVector3(height, radius, radius));
	m_collisionShapes->push_back(shape);
	body = createDynamicBody(mass, trans, shape);
	m_dynamicsWorld->addRigidBody(body, COL_FOLLICLE, follicleCollideWith);
	body->setActivationState(DISABLE_DEACTIVATION);

}

btRigidBody* Follicle::getBody() {
	return body;
}
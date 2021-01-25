#include "Follicle.h"

Follicle::Follicle(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes,
	               btTransform trans, btScalar radius, btScalar half_height, btScalar mass) {

	btCollisionShape* shape = new btCylinderShapeX(btVector3(half_height, radius, radius));
	m_collisionShapes->push_back(shape);
	body = createDynamicBody(mass, trans, shape);
	m_dynamicsWorld->addRigidBody(body, COL_FOLLICLE, follicleCollideWith);
	body->setActivationState(DISABLE_DEACTIVATION);

	m = mass;
	l = 2 * half_height;
}

btRigidBody* Follicle::getBody() {
	return body;
}

btScalar Follicle::getLength() {
	return l;
}

btScalar Follicle::getMass() {
	return m;
}
#include "my_pch.h"
#include "Follicle.h"

#include "Utility.h"

Follicle::Follicle(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes,
	               btTransform trans, btScalar radius, btScalar half_height, btScalar mass)
	: m_mass(mass), m_length(2*half_height) {

	m_shape = new btCylinderShapeX(btVector3(half_height, radius, radius));
	m_collisionShapes->push_back(m_shape);
	m_body = createDynamicBody(mass, trans, m_shape);
	m_dynamicsWorld->addRigidBody(m_body, COL_FOLLICLE, follicleCollideWith);
	m_body->setActivationState(DISABLE_DEACTIVATION);

}

Follicle::~Follicle() {
	// m_shape gets deleted with m_collishonShapes so no deletion here.
	// m_body gets deleted with m_dynamicsworld so no deletion here.
}

btRigidBody* Follicle::getBody() {
	return m_body;
}

btScalar Follicle::getLength() {
	return m_length;
}

btScalar Follicle::getMass() {
	return m_mass;
}
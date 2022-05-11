#include "my_pch.h"
#include "Follicle.h"

#include "Utility.h"


Follicle::Follicle(MystacialPad* pad, btTransform trans, btScalar radius, btScalar half_height, btScalar mass, int f)
	: m_pad(pad), m_mass(mass), m_length(2*half_height) {

	m_shape = new btCylinderShapeX(btVector3(half_height, radius, radius));
	getCollisionShapes()->push_back(m_shape);
	m_body = createDynamicBody(mass, trans, m_shape, 0 /*damping*/);
	getWorld()->addRigidBody(m_body, COL_FOLLICLE, follicleCollideWith);
	m_body->setActivationState(DISABLE_DEACTIVATION);
	m_info = new Follicle_info(f);
}

Follicle::~Follicle() {
	// m_shape gets deleted with m_collishonShapes so no deletion here.
	// m_body gets deleted with m_dynamicsworld so no deletion here.
	delete m_info;
}

btRigidBody* Follicle::getBody() const {
	return m_body;
}

btScalar Follicle::getLength() const {
	return m_length;
}

btScalar Follicle::getMass() const {
	return m_mass;
}

void* Follicle::getUserPointer() const {
	return m_body->getUserPointer();
}

void Follicle::setUserPointer(void* userPointer) {
	m_body->setUserPointer(userPointer);
}

btVector3 Follicle::getTopLocation() const {
	btVector3 Vb = btVector3(m_length / 2, 0, 0);
	return m_body->getCenterOfMassTransform() * Vb;
}

btVector3 Follicle::getBotLocation() const {
	btVector3 Vb = btVector3(-m_length /2, 0, 0);
	return m_body->getCenterOfMassTransform() * Vb;
}
#include "my_pch.h"
#include "Follicle.h"

#include "Utility.h"


Follicle::Follicle(MystacialPad* pad, btTransform trans, btScalar radius, btScalar half_height, btScalar mass, btScalar damping, int f)
	: m_pad(pad), m_mass(mass), m_length(2*half_height), m_damping(damping) {

	m_shape = new btCylinderShapeX(btVector3(half_height, radius, radius));
	getCollisionShapes()->push_back(m_shape);
	m_body = createDynamicBody(mass, trans, m_shape, m_damping/*damping*/);
	getWorld()->addRigidBody(m_body, COL_FOLLICLE, follicleCollideWith);
	m_body->setActivationState(DISABLE_DEACTIVATION);
	m_info = new Follicle_info(f);

	postUpdate();
}

Follicle::~Follicle() {
	// m_shape gets deleted with m_collishonShapes so no deletion here.
	// m_body gets deleted with m_dynamicsworld so no deletion here.
	delete m_info;
}

void Follicle::postUpdate() {
	topLoc = m_body->getCenterOfMassTransform() * btVector3(m_length * 0.5, 0, 0);
	botLoc = m_body->getCenterOfMassTransform() * btVector3(-m_length * 0.5, 0, 0);
	//calculate velocity: V + Omega x R
	btVector3 topR = m_body->getCenterOfMassTransform().getBasis() * btVector3(m_length * 0.5, 0, 0);
	btVector3 botR = m_body->getCenterOfMassTransform().getBasis() * btVector3(-m_length * 0.5, 0, 0);
	topVel = m_body->getLinearVelocity() + m_body->getAngularVelocity().cross(topR);
	botVel = m_body->getLinearVelocity() + m_body->getAngularVelocity().cross(botR);
}

btRigidBody* Follicle::getBody() const {
	return m_body;
}

btRigidBody* Follicle::getBody() {
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
	return topLoc;
}

btVector3 Follicle::getBotLocation() const {
	return botLoc;
}

btVector3& Follicle::getTopLocation() {
	return topLoc;
}

btVector3& Follicle::getBotLocation() {
	return botLoc;
}

btVector3 Follicle::getTopVelocity() const {
	return topVel;
}

btVector3 Follicle::getBotVelocity() const {
	return botVel;
}

btVector3& Follicle::getTopVelocity() {
	return topVel;
}

btVector3& Follicle::getBotVelocity() {
	return botVel;
}

btScalar Follicle::getHamiltonian() {
	// calculate the Hamiltonian as the sum of kinetic and potential energy
	btTransform Trans = m_body->getCenterOfMassTransform();
	btMatrix3x3 R = Trans.getBasis();
	btVector3 Ibody = m_body->getLocalInertia();
	btMatrix3x3 Ibody3x3(Ibody[0], 0, 0, 0, Ibody[1], 0, 0, 0, Ibody[2]);
	btMatrix3x3 I = R * Ibody3x3 * R.transpose();

	btVector3 AngularVel = m_body->getAngularVelocity();
	btScalar RotKineticEnergy = btVector3(I.tdotx(AngularVel), I.tdoty(AngularVel), I.tdotz(AngularVel)).dot(AngularVel) * 0.5;

	btScalar LinearKineticEnergy = m_body->getLinearVelocity().length2() * m_body->getMass() * 0.5;

	m_Hamiltonian = RotKineticEnergy + LinearKineticEnergy;
	return m_Hamiltonian;
}

void Follicle::debugDrawWhisker(btVector3& clr, btScalar lengthScale) const {
	btVector3 whiskerBase = topLoc;
	btVector3 whiskerDir = (topLoc - botLoc).normalized();

	btScalar halHeight = m_length * lengthScale * 0.5;
	btTransform whiskerTrans = createTransform(whiskerBase + halHeight * whiskerDir);
	whiskerTrans.setBasis(m_body->getCenterOfMassTransform().getBasis());
	m_pad->getWorld()->getDebugDrawer()->drawCylinder(0.05f, halHeight, 0, whiskerTrans, BLUE);
}
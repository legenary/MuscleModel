#include "IntrinsicMuscle.h"

IntrinsicSlingMuscle::IntrinsicSlingMuscle(btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	:Spring( body1, body2, frameInParent, frameInChild) {
	init(k, damping);
}

void IntrinsicSlingMuscle::init(btScalar k, btScalar damping) {
	initialize(k, damping);
}

void IntrinsicSlingMuscle::contract(btScalar ratio) {
	restLength = ratio * restLengthDefault;
}

void IntrinsicSlingMuscle::contract_HillType() {

}



IntrinsicObliqueMuscle::IntrinsicObliqueMuscle(btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	:Spring(body1, body2, frameInParent, frameInChild) {
	init(k, damping);
}

void IntrinsicObliqueMuscle::init(btScalar k, btScalar damping) {
	initialize(k, damping);
}

void IntrinsicObliqueMuscle::contract() {

}
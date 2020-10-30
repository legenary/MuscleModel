#include "IntrinsicMuscle.h"

IntrinsicSlingMuscle::IntrinsicSlingMuscle(btRigidBody* body1, btRigidBody* body2, btTransform frameInParent, btTransform frameInChild, btScalar k, btScalar damping)
	:Spring( body1, body2, frameInParent, frameInChild, k , damping) {
	init();
}

void IntrinsicSlingMuscle::init() {

}

void IntrinsicSlingMuscle::contract(btScalar ratio) {
	restLength = ratio * restLengthDefault;
}




IntrinsicObliqueMuscle::IntrinsicObliqueMuscle(btRigidBody* body1, btRigidBody* body2, btTransform frameInParent, btTransform frameInChild, btScalar k, btScalar damping)
	:Spring(body1, body2, frameInParent, frameInChild, k, damping) {
	init();
}

void IntrinsicObliqueMuscle::init() {

}

void IntrinsicObliqueMuscle::contract() {

}
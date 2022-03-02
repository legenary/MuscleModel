#include "my_pch.h"
#include "IntrinsicMuscle.h"

#include "Utility.h"
#include "Follicle.h"

IntrinsicSlingMuscle::IntrinsicSlingMuscle(btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	:TissueBetween( body1, body2, frameInParent, frameInChild, k, damping) {}


void IntrinsicSlingMuscle::contract(btScalar ratio) {
	m_restLength = ratio * m_restLengthDefault;
}

void IntrinsicSlingMuscle::contract_HillType() {
	
}



IntrinsicObliqueMuscle::IntrinsicObliqueMuscle(btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	:TissueBetween(body1, body2, frameInParent, frameInChild, k, damping) {
}


void IntrinsicObliqueMuscle::contract() {

}
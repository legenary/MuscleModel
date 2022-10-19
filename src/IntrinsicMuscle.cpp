#include "my_pch.h"
#include "IntrinsicMuscle.h"

#include "Utility.h"
#include "Follicle.h"


IntrinsicSlingMuscle::IntrinsicSlingMuscle(Simulation* sim, btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k)
	:Fiber(sim, body1, body2, frameInParent, frameInChild, k) {}

IntrinsicSlingMuscle::IntrinsicSlingMuscle(Simulation* sim, btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, int idx)
	:Fiber(sim, body1, body2, frameInParent, frameInChild, k, idx) {} 

//void IntrinsicSlingMuscle::contractTo(btScalar ratio) {
//	m_restLength = ratio * m_restLengthDefault;
//}




IntrinsicObliqueMuscle::IntrinsicObliqueMuscle(Simulation* sim, btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k)
	:Fiber(sim, body1, body2, frameInParent, frameInChild, k) {}


void IntrinsicObliqueMuscle::contract() {

}
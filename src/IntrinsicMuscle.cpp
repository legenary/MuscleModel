#include "my_pch.h"
#include "IntrinsicMuscle.h"

#include "Utility.h"
#include "Follicle.h"


IntrinsicSlingMuscle::IntrinsicSlingMuscle(Simulation* sim, btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	:Fiber(sim, body1, body2, frameInParent, frameInChild, k, damping) {}

//void IntrinsicSlingMuscle::contractTo(btScalar ratio) {
//	m_restLength = ratio * m_restLengthDefault;
//}




IntrinsicObliqueMuscle::IntrinsicObliqueMuscle(Simulation* sim, btRigidBody* body1, btRigidBody* body2, btTransform& frameInParent, btTransform& frameInChild, btScalar k, btScalar damping)
	:Fiber(sim, body1, body2, frameInParent, frameInChild, k, damping) {}


void IntrinsicObliqueMuscle::contract() {

}
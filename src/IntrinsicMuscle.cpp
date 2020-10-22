#include "IntrinsicMuscle.h"

IntrinsicSlingMuscle::IntrinsicSlingMuscle(Follicle* fol1, Follicle* fol2, btTransform frameInParent, btTransform frameInChild, btScalar k, btScalar damping)
	:Spring( fol1, fol2, frameInParent, frameInChild, k , damping) {
	init();
}

void IntrinsicSlingMuscle::init() {

}

void IntrinsicSlingMuscle::contract() {

}




IntrinsicObliqueMuscle::IntrinsicObliqueMuscle(Follicle* fol1, Follicle* fol2, btTransform frameInParent, btTransform frameInChild, btScalar k, btScalar damping)
	:Spring(fol1, fol2, frameInParent, frameInChild, k, damping) {
	init();
}

void IntrinsicObliqueMuscle::init() {

}

void IntrinsicObliqueMuscle::contract() {

}
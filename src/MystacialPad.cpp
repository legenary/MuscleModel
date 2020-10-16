#include "MystacialPad.h"

MystacialPad::MystacialPad(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {

	nFollicle = param->FOLLICLE_LOC_ORIENT.size();
	for (int f = 0; f < nFollicle; f++) {
		btVector3 this_pos = btVector3(param->FOLLICLE_LOC_ORIENT[f][0],
									   param->FOLLICLE_LOC_ORIENT[f][1],
									   param->FOLLICLE_LOC_ORIENT[f][2]);
		btVector3 this_ypr = btVector3(param->FOLLICLE_LOC_ORIENT[f][3],
									   -param->FOLLICLE_LOC_ORIENT[f][4],
									   param->FOLLICLE_LOC_ORIENT[f][5]);
		Follicle* follicle = new Follicle(m_dynamicsWorld, m_collisionShapes, this_pos, this_ypr, param->fol_radius, param->fol_height);
		m_follicleArray.push_back(follicle);
	}

}

int MystacialPad::getNumFollicles() {
	return nFollicle;
}

Follicle* MystacialPad::getFollicleByIndex(int idx) {
	return m_follicleArray[idx];
}



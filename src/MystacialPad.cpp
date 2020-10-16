#include "MystacialPad.h"

MystacialPad::MystacialPad(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {

	// create follicles
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

void MystacialPad::createLayer1(btDiscreteDynamicsWorld* world, Parameter* param) {

	// test
	btRigidBody* body0 = m_follicleArray[0]->getBody();
	btRigidBody* body1 = m_follicleArray[1]->getBody();

	btTransform frameInCurr = createTransform(btVector3(param->fol_height / 2, 0., 0.));
	btTransform frameInPrev = createTransform(btVector3(param->fol_height / 2, 0., 0.));

	btGeneric6DofSpringConstraint* constraint = new btGeneric6DofSpringConstraint(*body0, *body1, frameInPrev, frameInCurr, true);
	constraint->setLinearLowerLimit(btVector3(1, 1, 1));
	constraint->setLinearUpperLimit(btVector3(0, 0, 0));
	constraint->setAngularLowerLimit(btVector3(1, 1, 1));
	constraint->setAngularUpperLimit(btVector3(0, 0, 0));

	for (int i = 0; i < 3; i++) {
		constraint->enableSpring(i, true);
		constraint->setStiffness(i, 10);
		constraint->setDamping(i, 0);
		constraint->setEquilibriumPoint(i, 0);
	}
	//std::cout << constraint->getStiffness(1) << std::endl;
	world->addConstraint(constraint, false);

	body0->setLinearVelocity(btVector3(1, 0, 0));

}


int MystacialPad::getNumFollicles() {
	return nFollicle;
}

Follicle* MystacialPad::getFollicleByIndex(int idx) {
	return m_follicleArray[idx];
}



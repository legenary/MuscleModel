#include "MystacialPad.h"

MystacialPad::MystacialPad(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating follicles...";
	// create follicles
	nFollicle = param->FOLLICLE_LOC_ORIENT.size();
	std::cout << "Total Number " << nFollicle << "... ";
	for (int f = 0; f < nFollicle; f++) {
		btVector3 this_pos = btVector3(param->FOLLICLE_LOC_ORIENT[f][0],
									   param->FOLLICLE_LOC_ORIENT[f][1],
									   param->FOLLICLE_LOC_ORIENT[f][2]);
		btVector3 this_ypr = btVector3(param->FOLLICLE_LOC_ORIENT[f][3],
									   -param->FOLLICLE_LOC_ORIENT[f][4],
									   param->FOLLICLE_LOC_ORIENT[f][5]);
		btTransform this_trans = createTransform(this_pos, this_ypr);
		Follicle* follicle = new Follicle(m_dynamicsWorld, m_collisionShapes, this_trans, param->fol_radius, param->fol_height);
		m_follicleArray.push_back(follicle);
	}
	std::cout << "Done." << std::endl;
}

void MystacialPad::createLayer1(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating Layer1...";
	nSpringLayer1 = param->SPRING_HEX_MESH_INDEX.size();
	btTransform frameLayer1 = createTransform(btVector3(param->fol_height, 0., 0.));

	for (int f = 0; f < nSpringLayer1; f++) {
		Follicle* fol1 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][0]];
		Follicle* fol2 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][1]];
		Spring* springLayer1 = new Spring(fol1, fol2, frameLayer1, frameLayer1, param->k_layer1);
		m_dynamicsWorld->addConstraint(springLayer1->getConstraint(), true); // disable collision
		m_layer1.push_back(springLayer1);
	}
	std::cout << "Done." << std::endl;
}

void MystacialPad::createLayer2(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating Layer2...";
	nSpringLayer2 = param->SPRING_HEX_MESH_INDEX.size();
	btTransform frameLayer2 = createTransform(btVector3(-param->fol_height, 0., 0.));

	for (int f = 0; f < nSpringLayer1; f++) {
		Follicle* fol1 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][0]];
		Follicle* fol2 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][1]];
		Spring* springLayer2 = new Spring(fol1, fol2, frameLayer2, frameLayer2, param->k_layer2);
		m_dynamicsWorld->addConstraint(springLayer2->getConstraint(), true); // disable collision
		m_layer2.push_back(springLayer2);

	}
	std::cout << "Done." << std::endl;

}

void MystacialPad::test(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	// test
	btTransform T1 = createTransform(btVector3(0., 0., 0.), btVector3(0., PI/4, 0.));
	btTransform T2 = createTransform(btVector3(0., 0., 1.), btVector3(0., 0, 0.));

	Follicle* follicle1 = new Follicle(m_dynamicsWorld, m_collisionShapes, T1, param->fol_radius, param->fol_height);
	Follicle* follicle2 = new Follicle(m_dynamicsWorld, m_collisionShapes, T2, param->fol_radius, param->fol_height);
	m_follicleArray.push_back(follicle1);
	m_follicleArray.push_back(follicle2);
	btRigidBody* body1 = m_follicleArray[0]->getBody();
	btRigidBody* body2 = m_follicleArray[1]->getBody();

	btTransform frameInPrev = createTransform(btVector3(param->fol_height, 0., 0.));
	btTransform frameInCurr = createTransform(btVector3(param->fol_height, 0., 0.));

	btGeneric6DofSpringConstraint* constraint1 = new btGeneric6DofSpringConstraint(*body1, *body2, frameInPrev, frameInCurr, true);
	constraint1->setLinearLowerLimit(btVector3(1, 1, 1));
	constraint1->setLinearUpperLimit(btVector3(0, 0, 0));

	for (int i = 0; i < 3; i++) {
		constraint1->enableSpring(i, true);
		constraint1->setStiffness(i, 1);
		constraint1->setDamping(i, 1);	// guess: damping [0, 1] like restitution coefficient?
		constraint1->setEquilibriumPoint(i);	// rest length initialization
	}

	
	//std::cout << constraint->getStiffness(1) << std::endl;
	m_dynamicsWorld->addConstraint(constraint1, true); //true = no collision; false = collision


	//body2->setCenterOfMassTransform(createTransform(btVector3(0., 0., 1.), btVector3(0, PI/6, 0.)));
	std::cout << constraint1->getEquilibriumPoint(0) << "  "
		<< constraint1->getEquilibriumPoint(1) << "  "
		<< constraint1->getEquilibriumPoint(2) << "  " << std::endl;



}

void MystacialPad::updateLayer1() {
	for (int i = 1; i < nSpringLayer1; i++) {
		m_layer1[i]->update();
	}
}

void MystacialPad::updateLayer2() {
	for (int i = 1; i < nSpringLayer2; i++) {
		m_layer2[i]->update();
	}
}

int MystacialPad::getNumFollicles() {
	return nFollicle;
}

Follicle* MystacialPad::getFollicleByIndex(int idx) {
	return m_follicleArray[idx];
}



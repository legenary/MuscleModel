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

	for (int f = 0; f < nSpringLayer2; f++) {
		Follicle* fol1 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][0]];
		Follicle* fol2 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][1]];
		Spring* springLayer2 = new Spring(fol1, fol2, frameLayer2, frameLayer2, param->k_layer2);
		m_dynamicsWorld->addConstraint(springLayer2->getConstraint(), true); // disable collision
		m_layer2.push_back(springLayer2);

	}
	std::cout << "Done." << std::endl;
}

void MystacialPad::update() {
	for (int i = 0; i < nSpringLayer1; i++) {
		m_layer1[i]->update();
	}
	for (int i = 0; i < nSpringLayer2; i++) {
		m_layer2[i]->update();
	}
}

void MystacialPad::update_test(btDiscreteDynamicsWorld* m_dynamicsWorld, int DEBUG) {
	if (DEBUG) {
		for (int i = 0; i < nSpringLayer1; i++) {
			m_layer1[i]->test(m_dynamicsWorld);
			m_layer2[i]->test(m_dynamicsWorld);

			printf("%.2f ", m_layer1[i]->getLength() / m_layer1[i]->getRestLength());
			printf("%.2f ", m_layer2[i]->getLength() / m_layer2[i]->getRestLength());

		}
		std::cout << std::endl;
		
	}
}

int MystacialPad::getNumFollicles() const {
	return nFollicle;
}

Follicle* MystacialPad::getFollicleByIndex(int idx) {
	return m_follicleArray[idx];
}




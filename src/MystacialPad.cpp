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

void MystacialPad::createLayer1(btDiscreteDynamicsWorld* m_dynamicsWorld, Parameter* param) {
	std::cout << "Creating Layer1...";
	nSpringLayer1 = param->SPRING_HEX_MESH_INDEX.size();
	btTransform frameLayer1 = createTransform(btVector3(param->fol_height, 0., 0.));

	for (int f = 0; f < nSpringLayer1; f++) {
		Follicle* fol1 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][0]];
		Follicle* fol2 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][1]];
		Spring* springLayer1 = new Spring(fol1, fol2, frameLayer1, frameLayer1, param->k_layer1, param->damping);
		m_dynamicsWorld->addConstraint(springLayer1->getConstraint(), true); // disable collision
		m_layer1.push_back(springLayer1);
	}
	std::cout << "Done." << std::endl;
}

void MystacialPad::createLayer2(btDiscreteDynamicsWorld* m_dynamicsWorld, Parameter* param) {
	std::cout << "Creating Layer2...";
	nSpringLayer2 = param->SPRING_HEX_MESH_INDEX.size();
	btTransform frameLayer2 = createTransform(btVector3(-param->fol_height, 0., 0.));

	for (int f = 0; f < nSpringLayer2; f++) {
		Follicle* fol1 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][0]];
		Follicle* fol2 = m_follicleArray[param->SPRING_HEX_MESH_INDEX[f][1]];
		Spring* springLayer2 = new Spring(fol1, fol2, frameLayer2, frameLayer2, param->k_layer2, param->damping);
		m_dynamicsWorld->addConstraint(springLayer2->getConstraint(), true); // disable collision
		m_layer2.push_back(springLayer2);

	}
	std::cout << "Done." << std::endl;
}

void MystacialPad::createAnchor(btDiscreteDynamicsWorld* m_dynamicsWorld, Parameter* param) {
	std::cout << "Creating Follicle Anchoring...";
	btTransform frameAnchor = createTransform(btVector3(param->fol_height, 0., 0.));

	for (int f = 0; f < nFollicle; f++) {
		Follicle* fol = m_follicleArray[f];
		Spring* springAnchor = new Spring(fol, frameAnchor, param->k_anchor, param->damping, false);	// true = linear spring
																										// false = torsional spring
		m_dynamicsWorld->addConstraint(springAnchor->getConstraint(), true); // disable collision
		m_anchor.push_back(springAnchor);
	}

	std::cout << "Done." << std::endl;
}

void MystacialPad::createIntrinsicSlingMuscle(btDiscreteDynamicsWorld* m_dynamicsWorld, Parameter* param) {
	std::cout << "Creating intrinsic sling muscles...";
	nSpringISM = param->INTRINSIC_SLING_MUSCLE_INDEX.size();
	btTransform frameC = createTransform(btVector3(param->fol_height, 0., 0.));
	btTransform frameR = createTransform(btVector3(-param->fol_height, 0., 0.));

	for (int m = 0; m < nSpringISM; m++) {
		Follicle* folC = m_follicleArray[param->INTRINSIC_SLING_MUSCLE_INDEX[m][0]];
		Follicle* folR = m_follicleArray[param->INTRINSIC_SLING_MUSCLE_INDEX[m][1]];
		IntrinsicSlingMuscle* muscle = new IntrinsicSlingMuscle(folC, folR, frameC, frameR, param->k_ISM, param->damping);
		m_dynamicsWorld->addConstraint(muscle->getConstraint(), true); // disable collision
		m_ISMArray.push_back(muscle);

	}
	std::cout << "Done." << std::endl;
}

void MystacialPad::contract(int m_step, Parameter* param) {
	// contract intrinsic sling muscle
	int TrajectoryLength = param->INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY.size();
	int step = (m_step <= TrajectoryLength) ? (m_step - 1) : (TrajectoryLength - 1);
	if (param->contractISM) {
		for (int i = 0; i < nSpringISM; i++) {
			m_ISMArray[i]->setRestLength(param->INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY[step][0]);
		}
	}
}

void MystacialPad::update() {
	// only linear springs need update
	// torsional springs don't
	for (int i = 0; i < nSpringLayer1; i++) {
		m_layer1[i]->update();
	}
	for (int i = 0; i < nSpringLayer2; i++) {
		m_layer2[i]->update();
	}
	for (int i = 0; i < nSpringISM; i++) {
		m_ISMArray[i]->update();
	}
}

void MystacialPad::debugDraw(btDiscreteDynamicsWorld* m_dynamicsWorld, int DEBUG) {
	if (DEBUG) { // debug draw springs
		for (int i = 0; i < m_layer1.size(); i++) {
			m_layer1[i]->debugDraw(m_dynamicsWorld);
		}
		for (int i = 0; i < m_layer2.size(); i++) {
			m_layer2[i]->debugDraw(m_dynamicsWorld);
		}
		for (int i = 0; i < m_ISMArray.size(); i++) {
			m_ISMArray[i]->debugDraw(m_dynamicsWorld, btVector3(0., 0., 1.));
		}
	}
}

int MystacialPad::getNumFollicles() const {
	return nFollicle;
}

Follicle* MystacialPad::getFollicleByIndex(int idx) {
	return m_follicleArray[idx];
}




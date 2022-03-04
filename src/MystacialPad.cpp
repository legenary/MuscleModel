#include "my_pch.h"
#include "MystacialPad.h"

#include "Follicle.h"
#include "Utility.h"
#include "Parameter.h"
#include "Tissue.h"
#include "IntrinsicMuscle.h"
#include "ExtrinsicMuscle.h"

MystacialPad::MystacialPad(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param)
	: nFollicle(0), nTissueLayer1(0), nTissueLayer2(0), nTissueAnchor(0), nTissueISM(0)
	, m_nasolabialis(nullptr), m_maxillolabialis(nullptr), m_NS(nullptr)
	, m_PMS(nullptr), m_PMI(nullptr), m_PIP(nullptr), m_PM(nullptr) {
	std::cout << "Creating follicles...";
	// create follicles
	nFollicle = param->FOLLICLE_POS_ORIENT_LEN_VOL.size();
	std::cout << "Total Number " << nFollicle << "... ";
	for (int f = 0; f < nFollicle; f++) {
		btVector3 this_pos = btVector3(param->FOLLICLE_POS_ORIENT_LEN_VOL[f][0],
									   param->FOLLICLE_POS_ORIENT_LEN_VOL[f][1],
									   param->FOLLICLE_POS_ORIENT_LEN_VOL[f][2]);
		btVector3 this_ypr = btVector3(param->FOLLICLE_POS_ORIENT_LEN_VOL[f][3],
									   -param->FOLLICLE_POS_ORIENT_LEN_VOL[f][4],
									   param->FOLLICLE_POS_ORIENT_LEN_VOL[f][5]);
		btScalar this_len = param->FOLLICLE_POS_ORIENT_LEN_VOL[f][6];	// length already in mm
		btScalar this_mass = param->FOLLICLE_POS_ORIENT_LEN_VOL[f][7];	// volume already in mm^3
		btTransform this_trans = createTransform(this_pos, this_ypr);
		Follicle* follicle = new Follicle(m_dynamicsWorld, m_collisionShapes, 
			this_trans, param->fol_radius, this_len/2, this_mass, f);
		m_follicleArray.push_back(follicle);
	}
	std::cout << "Done.\n";
}

MystacialPad::~MystacialPad() {
	freeAlignedObjectArray(m_follicleArray);
	freeAlignedObjectArray(m_layer1);
	freeAlignedObjectArray(m_layer2);
	freeAlignedObjectArray(m_layer3);
	freeAlignedObjectArray(m_anchor);
	freeAlignedObjectArray(m_ISMArray);

	delete m_nasolabialis;
	delete m_maxillolabialis;
	delete m_NS;
	delete m_PMS;
	delete m_PMI;
	delete m_PIP;
	delete m_PM;
}

void MystacialPad::createLayer1(btDiscreteDynamicsWorld* m_dynamicsWorld, Parameter* param) {
	std::cout << "Creating Layer1...";
	nTissueLayer1 = param->SPRING_HEX_MESH_IDX.size();

	for (int s = 0; s < nTissueLayer1; s++) {
		Follicle* fol1 = m_follicleArray[param->SPRING_HEX_MESH_IDX[s][0]];
		Follicle* fol2 = m_follicleArray[param->SPRING_HEX_MESH_IDX[s][1]];
		btTransform frameLayer1fol1 = createTransform(btVector3(param->FOLLICLE_POS_ORIENT_LEN_VOL[param->SPRING_HEX_MESH_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameLayer1fol2 = createTransform(btVector3(param->FOLLICLE_POS_ORIENT_LEN_VOL[param->SPRING_HEX_MESH_IDX[s][1]][6] / 2, 0., 0.));

		btScalar k_eq = 300;
		btScalar k_this = k_eq / 2;
		btScalar damping_this = getCriticalDampingRatio(fol1->getMass(), fol2->getMass(), k_eq) * 2;
		Tissue* springLayer1 = new Tissue(fol1->getBody(), fol2->getBody(), frameLayer1fol1, frameLayer1fol2, k_this, damping_this);

		m_dynamicsWorld->addConstraint(springLayer1->getConstraint(), true); // disable collision
		m_layer1.push_back(springLayer1);
	}
	std::cout << "Done.\n";
}

void MystacialPad::createLayer2(btDiscreteDynamicsWorld* m_dynamicsWorld, Parameter* param) {
	std::cout << "Creating Layer2...";
	nTissueLayer2 = param->SPRING_HEX_MESH_IDX.size();

	for (int s = 0; s < nTissueLayer2; s++) {
		Follicle* fol1 = m_follicleArray[param->SPRING_HEX_MESH_IDX[s][0]];
		Follicle* fol2 = m_follicleArray[param->SPRING_HEX_MESH_IDX[s][1]];
		btTransform frameLayer2fol1 = createTransform(btVector3(-param->FOLLICLE_POS_ORIENT_LEN_VOL[param->SPRING_HEX_MESH_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameLayer2fol2 = createTransform(btVector3(-param->FOLLICLE_POS_ORIENT_LEN_VOL[param->SPRING_HEX_MESH_IDX[s][1]][6] / 2, 0., 0.));

		btScalar k_eq = 300;
		btScalar k_this = k_eq / 2;
		btScalar damping_this = getCriticalDampingRatio(fol1->getMass(), fol2->getMass(), k_eq) * 2;
		Tissue* springLayer2 = new Tissue(fol1->getBody(), fol2->getBody(), frameLayer2fol1, frameLayer2fol2, k_this, damping_this);

		m_dynamicsWorld->addConstraint(springLayer2->getConstraint(), true); // disable collision
		m_layer2.push_back(springLayer2);

	}
	std::cout << "Done.\n";
}

void MystacialPad::createAnchor(btDiscreteDynamicsWorld* m_dynamicsWorld, Parameter* param) {
	std::cout << "Creating Follicle Anchoring...";

	nTissueAnchor = nFollicle;
	for (int f = 0; f < nTissueAnchor; f++) {
		Follicle* fol = m_follicleArray[f];
		btTransform frameAnchor = createTransform(btVector3(param->FOLLICLE_POS_ORIENT_LEN_VOL[f][6] / 2, 0., 0.));
		Tissue* tissueAnchor = new Tissue(fol->getBody(), frameAnchor, param->k_anchor, param->damping);	// this is a linear + torsional spring
																										
		m_dynamicsWorld->addConstraint(tissueAnchor->getConstraint(), true); // disable collision
		m_anchor.push_back(tissueAnchor);
	}

	std::cout << "Done.\n";
}

void MystacialPad::createIntrinsicSlingMuscle(btDiscreteDynamicsWorld* m_dynamicsWorld, Parameter* param) {
	std::cout << "Creating intrinsic sling muscles...";
	nTissueISM = param->INTRINSIC_SLING_MUSCLE_IDX.size();

	for (int s = 0; s < nTissueISM; s++) {
		Follicle* folC = m_follicleArray[param->INTRINSIC_SLING_MUSCLE_IDX[s][0]];
		Follicle* folR = m_follicleArray[param->INTRINSIC_SLING_MUSCLE_IDX[s][1]];
		btTransform frameC = createTransform(btVector3(param->FOLLICLE_POS_ORIENT_LEN_VOL[param->INTRINSIC_SLING_MUSCLE_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameR = createTransform(btVector3(-param->FOLLICLE_POS_ORIENT_LEN_VOL[param->INTRINSIC_SLING_MUSCLE_IDX[s][1]][6] / 2 * 0.4, 0., 0.)); // 70% of rostral member
		IntrinsicSlingMuscle* muscle = new IntrinsicSlingMuscle(folC->getBody(), folR->getBody(), frameC, frameR, param->k_ISM, param->damping);
		m_dynamicsWorld->addConstraint(muscle->getConstraint(), true); // disable collision
		m_ISMArray.push_back(muscle);

	}
	std::cout << "Done.\n";
}

void MystacialPad::contractIntrinsicSlingMuscle(int m_step, Parameter* param) {
	// contract intrinsic sling muscle
	int TrajectoryLength = param->INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY.size();
	int step = (m_step <= TrajectoryLength) ? (m_step - 1) : (TrajectoryLength - 1);

	for (int i = 0; i < nTissueISM; i++) {
		m_ISMArray[i]->contract(param->INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY[step][0]);
	}
}

void MystacialPad::contractIntrinsicSlingMuscle(int m_step, Parameter* param, std::vector<int>& those) {
	// contract intrinsic sling muscle
	int TrajectoryLength = param->INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY.size();
	int step = (m_step <= TrajectoryLength) ? (m_step - 1) : (TrajectoryLength - 1);

	for (auto& that : those) {
		m_ISMArray[that]->contract(param->INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY[step][0]);
	}
}

void MystacialPad::createNasolabialis(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating extrinsic muscles: M.Nasolabialis ...";
	m_nasolabialis = new ExtrinsicMuscle(m_dynamicsWorld, m_collisionShapes, param, m_follicleArray,
		param->NASOLABIALIS_NODE_POS, param->NASOLABIALIS_CONSTRUCTION_IDX, param->NASOLABIALIS_INSERTION_IDX, heightPlaceHolder);
	std::cout << "Done.\n";
}

void MystacialPad::contractNasolabialis(int m_step, Parameter* param) {
	// contract nasolabialis extrinsic muscle
	int TrajectoryLength = param->NASOLABIALIS_CONTRACTION_TRAJECTORY.size();
	int step = (m_step <= TrajectoryLength) ? (m_step - 1) : (TrajectoryLength - 1);

	m_nasolabialis->contract(param->NASOLABIALIS_CONTRACTION_TRAJECTORY[step][0]);
}

void MystacialPad::createMaxillolabialis(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating extrinsic muscles: M.Maxillolabialis ...";
	m_maxillolabialis = new ExtrinsicMuscle(m_dynamicsWorld, m_collisionShapes, param, m_follicleArray,
		param->MAXILLOLABIALIS_NODE_POS, param->MAXILLOLABIALIS_CONSTRUCTION_IDX, param->MAXILLOLABIALIS_INSERTION_IDX, heightPlaceHolder);
	std::cout << "Done." << std::endl;
}

void MystacialPad::contractMaxillolabialis(int m_step, Parameter* param) {
	// contract nasolabialis extrinsic muscle
	int TrajectoryLength = param->MAXILLOLABIALIS_CONTRACTION_TRAJECTORY.size();
	int step = (m_step <= TrajectoryLength) ? (m_step - 1) : (TrajectoryLength - 1);

	m_maxillolabialis->contract(param->MAXILLOLABIALIS_CONTRACTION_TRAJECTORY[step][0]);
}

void MystacialPad::createNasolabialisSuperficialis(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating extrinsic muscles: M.Nasolabialis superficialis ...";
	m_NS = new ExtrinsicMuscle(m_dynamicsWorld, m_collisionShapes, param, m_follicleArray,
		param->NASOLABIALIS_SUPERFICIALIS_NODE_POS, param->NASOLABIALIS_SUPERFICIALIS_CONSTRUCTION_IDX, param->NASOLABIALIS_SUPERFICIALIS_INSERTION_IDX, heightPlaceHolder);
	std::cout << "Done.\n";
}

void MystacialPad::createParsMediaSuperior(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating extrinsci muscles: Pars media superior of M. Nasolabialis profundus...";
	m_PMS = new ExtrinsicMuscle(m_dynamicsWorld, m_collisionShapes, param, m_follicleArray, param->PARS_MEDIA_SUPERIOR_NODE_POS, 
		param->PARS_MEDIA_SUPERIOR_CONSTRUCTION_IDX, param->PARS_MEDIA_SUPERIOR_INSERTION_IDX, param->PARS_MEDIA_SUPERIOR_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsMediaInferior(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating extrinsci muscles: Pars media inferior of M. Nasolabialis profundus...";
	m_PMI = new ExtrinsicMuscle(m_dynamicsWorld, m_collisionShapes, param, m_follicleArray, param->PARS_MEDIA_INFERIOR_NODE_POS, 
		param->PARS_MEDIA_INFERIOR_CONSTRUCTION_IDX, param->PARS_MEDIA_INFERIOR_INSERTION_IDX, param->PARS_MEDIA_INFERIOR_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsInternaProfunda(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating extrinsci muscles: Pars interna profunda of M. Nasolabialis profundus...";
	m_PIP = new ExtrinsicMuscle(m_dynamicsWorld, m_collisionShapes, param, m_follicleArray, param->PARS_INTERNA_PROFUNDA_NODE_POS,
		param->PARS_INTERNA_PROFUNDA_CONSTRUCTION_IDX, param->PARS_INTERNA_PROFUNDA_INSERTION_IDX, param->PARS_INTERNA_PROFUNDA_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsMaxillaris(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param) {
	std::cout << "Creating extrinsci muscles: Pars maxillaris (superficialis and profunda combined) of M. Nasolabialis profundus...";
	m_PM = new ExtrinsicMuscle(m_dynamicsWorld, m_collisionShapes, param, m_follicleArray, param->PARS_MAXILLARIS_NODE_POS,
		param->PARS_MAXILLARIS_CONSTRUCTION_IDX, param->PARS_MAXILLARIS_INSERTION_IDX, param->PARS_MAXILLARIS_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::update() {
	// only linear springs need update
	// torsional springs don't
	for (int i = 0; i < nTissueLayer1; i++) {
		m_layer1[i]->update();
	}
	for (int i = 0; i < nTissueLayer2; i++) {
		m_layer2[i]->update();
	}
	for (int i = 0; i < nTissueAnchor; i++) {
		m_anchor[i]->update();
	}

	for (int i = 0; i < nTissueISM; i++) {
		m_ISMArray[i]->update();
	}

	if (m_nasolabialis != nullptr)		m_nasolabialis->update();
	if (m_maxillolabialis != nullptr)	m_maxillolabialis->update();
	if (m_NS != nullptr)				m_NS->update();
	if (m_PMS != nullptr)				m_PMS->update();
	if (m_PMI != nullptr)				m_PMI->update();
	if (m_PIP != nullptr)				m_PIP->update();
	if (m_PM != nullptr)				m_PM->update();
}

void MystacialPad::debugDraw(btDiscreteDynamicsWorld* m_dynamicsWorld, int DEBUG) {
	if (DEBUG) { // debug draw springs
		for (int i = 0; i < m_layer1.size(); i++) {
			m_layer1[i]->debugDraw(m_dynamicsWorld, btVector3(1., 0., 0.), true);
		}
		for (int i = 0; i < m_layer2.size(); i++) {
			m_layer2[i]->debugDraw(m_dynamicsWorld, btVector3(1., 0., 0.), true);
		}
		for (int i = 0; i < m_anchor.size(); i++) {
			m_anchor[i]->debugDraw(m_dynamicsWorld, btVector3(1., 0., 0.), true);
		}

		//for (int i = 0; i < m_ISMArray.size(); i++) {
		//	m_ISMArray[i]->debugDraw(m_dynamicsWorld, btVector3(0., 0., 1.));
		//}
		//m_nasolabialis->debugDraw(m_dynamicsWorld);
		//m_maxillolabialis->debugDraw(m_dynamicsWorld); 
		//m_NS->debugDraw(m_dynamicsWorld, btVector3(0., 0., 1.));
		//m_PMS->debugDraw(m_dynamicsWorld, btVector3(0., 0., 1.));
		//m_PMI->debugDraw(m_dynamicsWorld, btVector3(0., 0., 1.));
		//m_PIP->debugDraw(m_dynamicsWorld, btVector3(0., 1., 0.));
		//m_PM->debugDraw(m_dynamicsWorld, btVector3(0., 1., 0.));
	}
}

int MystacialPad::getNumFollicles() const {
	return nFollicle;
}

Follicle* MystacialPad::getFollicleByIndex(int idx) {
	return m_follicleArray[idx];
}




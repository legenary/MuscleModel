#include "my_pch.h"
#include "MystacialPad.h"

#include "Utility.h"
#include "Parameter.h"
#include "Follicle.h"
#include "Tissue.h"
#include "IntrinsicMuscle.h"
#include "ExtrinsicMuscle.h"


MystacialPad::MystacialPad(Simulation* sim, Parameter* param)
	: m_sim(sim), m_parameter(param)
	, nFollicle(0), nTissueLayer1(0), nTissueLayer2(0), nTissueAnchor(0), nISM(0)
	, m_nasolabialis(nullptr), m_maxillolabialis(nullptr), m_NS(nullptr)
	, m_PMS(nullptr), m_PMI(nullptr), m_PIP(nullptr), m_PMP(nullptr) {
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
		Follicle* follicle = new Follicle(this, this_trans, param->fol_radius, this_len/2, this_mass, f);
		follicle->setUserPointer(follicle->getInfo());
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
	delete m_PMP;
}

void MystacialPad::createLayer1() {
	std::cout << "Creating Layer1...";
	nTissueLayer1 = m_parameter->SPRING_HEX_MESH_IDX.size();

	for (int s = 0; s < nTissueLayer1; s++) {
		Follicle* fol1 = m_follicleArray[m_parameter->SPRING_HEX_MESH_IDX[s][0]];
		Follicle* fol2 = m_follicleArray[m_parameter->SPRING_HEX_MESH_IDX[s][1]];
		btTransform frameLayer1fol1 = createTransform(btVector3(m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->SPRING_HEX_MESH_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameLayer1fol2 = createTransform(btVector3(m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->SPRING_HEX_MESH_IDX[s][1]][6] / 2, 0., 0.));

		btScalar k_eq = m_parameter->k_layer;
		btScalar k_this = k_eq / 2;
		btScalar damping_this = getCriticalDampingRatio(fol1->getMass(), fol2->getMass(), k_eq) * 2;
		Tissue* springLayer1 = new Tissue(m_sim, fol1->getBody(), fol2->getBody(), frameLayer1fol1, frameLayer1fol2, k_this, damping_this);

		getWorld()->addConstraint(springLayer1->getConstraint(), true); // disable collision
		m_layer1.push_back(springLayer1);
	}
	std::cout << "Done.\n";
}

void MystacialPad::createLayer2() {
	std::cout << "Creating Layer2...";
	nTissueLayer2 = m_parameter->SPRING_HEX_MESH_IDX.size();

	for (int s = 0; s < nTissueLayer2; s++) {
		Follicle* fol1 = m_follicleArray[m_parameter->SPRING_HEX_MESH_IDX[s][0]];
		Follicle* fol2 = m_follicleArray[m_parameter->SPRING_HEX_MESH_IDX[s][1]];
		btTransform frameLayer2fol1 = createTransform(btVector3(-m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->SPRING_HEX_MESH_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameLayer2fol2 = createTransform(btVector3(-m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->SPRING_HEX_MESH_IDX[s][1]][6] / 2, 0., 0.));

		btScalar k_eq = m_parameter->k_layer;
		btScalar k_this = k_eq / 2;
		btScalar damping_this = getCriticalDampingRatio(fol1->getMass(), fol2->getMass(), k_eq) * 2;
		Tissue* springLayer2 = new Tissue(m_sim, fol1->getBody(), fol2->getBody(), frameLayer2fol1, frameLayer2fol2, k_this, damping_this);

		getWorld()->addConstraint(springLayer2->getConstraint(), true); // disable collision
		m_layer2.push_back(springLayer2);

	}
	std::cout << "Done.\n";
}

void MystacialPad::createAnchor() {
	std::cout << "Creating Follicle Anchoring...";

	nTissueAnchor = nFollicle;
	for (int f = 0; f < nTissueAnchor; f++) {
		Follicle* fol = m_follicleArray[f];
		btTransform frameAnchor = createTransform(btVector3(m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[f][6] / 2, 0., 0.));
		Tissue* tissueAnchor = new Tissue(m_sim, fol->getBody(), frameAnchor, m_parameter->k_anchor, m_parameter->dmp_anchor);	// this is a linear + torsional spring
																										
		getWorld()->addConstraint(tissueAnchor->getConstraint(), true); // disable collision
		m_anchor.push_back(tissueAnchor);
	}

	std::cout << "Done.\n";
}

void MystacialPad::createIntrinsicSlingMuscle() {
	std::cout << "Creating intrinsic sling muscles...";
	nISM = m_parameter->INTRINSIC_SLING_MUSCLE_IDX.size();

	for (int s = 0; s < nISM; s++) {
		Follicle* folC = m_follicleArray[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[s][0]];
		Follicle* folR = m_follicleArray[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[s][1]];
		btTransform frameC = createTransform(btVector3(m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameR = createTransform(btVector3(-m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[s][1]][6] / 2 * 0.4, 0., 0.)); // 70% of rostral member
		IntrinsicSlingMuscle* muscle = new IntrinsicSlingMuscle(m_sim, folC->getBody(), folR->getBody(), frameC, frameR, m_parameter->k_ISM, 1 /*no damping*/);
		getWorld()->addConstraint(muscle->getConstraint(), true); // disable collision
		m_ISMArray.push_back(muscle);

	}
	std::cout << "Done.\n";
}

void MystacialPad::createNasolabialis() {
	std::cout << "Creating extrinsic muscles: M.Nasolabialis ...";
	m_nasolabialis = new ExtrinsicMuscle(m_sim, m_parameter, m_follicleArray,
		m_parameter->NASOLABIALIS_NODE_POS, m_parameter->NASOLABIALIS_CONSTRUCTION_IDX, m_parameter->NASOLABIALIS_INSERTION_IDX, heightPlaceHolder);
	std::cout << "Done.\n";
}


void MystacialPad::createMaxillolabialis() {
	std::cout << "Creating extrinsic muscles: M.Maxillolabialis ...";
	m_maxillolabialis = new ExtrinsicMuscle(m_sim, m_parameter, m_follicleArray,
		m_parameter->MAXILLOLABIALIS_NODE_POS, m_parameter->MAXILLOLABIALIS_CONSTRUCTION_IDX, m_parameter->MAXILLOLABIALIS_INSERTION_IDX, heightPlaceHolder);
	std::cout << "Done." << std::endl;
}

void MystacialPad::contractMuscle(Muscle mus, btScalar ratio) {
	switch (mus) {
	case INTRINSIC:
		for (int s = 0; s < nISM; s++)
			m_ISMArray[s]->contractTo(ratio);
		break;
	case N:
		m_nasolabialis->contractTo(ratio);
		break;
	case M:
		m_maxillolabialis->contractTo(ratio);
		break;
	case NS:
		m_NS->contractTo(ratio);
		break;
	case PMS:
		m_PMS->contractTo(ratio);
		break;
	case PMI:
		m_PMI->contractTo(ratio);
		break;
	case PIP:
		m_PIP->contractTo(ratio);
		break;
	case PMP:
		m_PMP->contractTo(ratio);
		break;
	}
}

void MystacialPad::createNasolabialisSuperficialis() {
	std::cout << "Creating extrinsic muscles: M.Nasolabialis superficialis ...";
	m_NS = new ExtrinsicMuscle(m_sim, m_parameter, m_follicleArray,
		m_parameter->NASOLABIALIS_SUPERFICIALIS_NODE_POS, m_parameter->NASOLABIALIS_SUPERFICIALIS_CONSTRUCTION_IDX, m_parameter->NASOLABIALIS_SUPERFICIALIS_INSERTION_IDX, heightPlaceHolder,
		std::set<int>{ 0, 12, 13, 14, 15, 16 });
	std::cout << "Done.\n";
}

void MystacialPad::createParsMediaSuperior() {
	std::cout << "Creating extrinsci muscles: Pars media superior of M. Nasolabialis profundus...";
	m_PMS = new ExtrinsicMuscle(m_sim, m_parameter, m_follicleArray, m_parameter->PARS_MEDIA_SUPERIOR_NODE_POS,
		m_parameter->PARS_MEDIA_SUPERIOR_CONSTRUCTION_IDX, m_parameter->PARS_MEDIA_SUPERIOR_INSERTION_IDX, m_parameter->PARS_MEDIA_SUPERIOR_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsMediaInferior() {
	std::cout << "Creating extrinsci muscles: Pars media inferior of M. Nasolabialis profundus...";
	m_PMI = new ExtrinsicMuscle(m_sim, m_parameter, m_follicleArray, m_parameter->PARS_MEDIA_INFERIOR_NODE_POS,
		m_parameter->PARS_MEDIA_INFERIOR_CONSTRUCTION_IDX, m_parameter->PARS_MEDIA_INFERIOR_INSERTION_IDX, m_parameter->PARS_MEDIA_INFERIOR_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsInternaProfunda() {
	std::cout << "Creating extrinsci muscles: Pars interna profunda of M. Nasolabialis profundus...";
	m_PIP = new ExtrinsicMuscle(m_sim, m_parameter, m_follicleArray, m_parameter->PARS_INTERNA_PROFUNDA_NODE_POS,
		m_parameter->PARS_INTERNA_PROFUNDA_CONSTRUCTION_IDX, m_parameter->PARS_INTERNA_PROFUNDA_INSERTION_IDX, m_parameter->PARS_INTERNA_PROFUNDA_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsMaxillarisProfunda() {
	std::cout << "Creating extrinsci muscles: Pars maxillaris (superficialis and profunda combined) of M. Nasolabialis profundus...";
	m_PMP = new ExtrinsicMuscle(m_sim, m_parameter, m_follicleArray, m_parameter->PARS_MAXILLARIS_NODE_POS,
		m_parameter->PARS_MAXILLARIS_CONSTRUCTION_IDX, m_parameter->PARS_MAXILLARIS_INSERTION_IDX, m_parameter->PARS_MAXILLARIS_INSERTION_HEIGHT);
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

	for (int i = 0; i < nISM; i++) {
		m_ISMArray[i]->update();
	}

	if (m_nasolabialis != nullptr)		m_nasolabialis->update();
	if (m_maxillolabialis != nullptr)	m_maxillolabialis->update();
	if (m_NS != nullptr)				m_NS->update();
	if (m_PMS != nullptr)				m_PMS->update();
	if (m_PMI != nullptr)				m_PMI->update();
	if (m_PIP != nullptr)				m_PIP->update();
	if (m_PMP != nullptr)				m_PMP->update();
}

void MystacialPad::debugDraw() {
	//for (int i = 0; i < m_layer1.size(); i++) {
	//	m_layer1[i]->debugDraw(btVector3(1., 0., 0.), true);
	//}
	//for (int i = 0; i < m_layer2.size(); i++) {
	//	m_layer2[i]->debugDraw(btVector3(1., 0., 0.), true);
	//}
	for (int i = 0; i < m_anchor.size(); i++) {
		m_anchor[i]->debugDraw(btVector3(1., 0., 0.), true);
	}

	for (int i = 0; i < m_ISMArray.size(); i++) {
		m_ISMArray[i]->debugDraw(btVector3(0., 0., 1.), false);
	}
	//m_nasolabialis->debugDraw(btVector3(0., 0., 1.));
	//m_maxillolabialis->debugDraw(btVector3(0., 0., 1.));
	//m_NS->debugDraw(btVector3(0., 0., 1.));
	//m_PMS->debugDraw(btVector3(0., 0., 1.));
	//m_PMI->debugDraw(btVector3(0., 0., 1.));
	//m_PIP->debugDraw(btVector3(0., 1., 0.));
	//m_PMP->debugDraw(btVector3(0., 1., 0.));
}

int MystacialPad::getNumFollicles() const {
	return nFollicle;
}

Follicle* MystacialPad::getFollicleByIndex(int idx) {
	return m_follicleArray[idx];
}




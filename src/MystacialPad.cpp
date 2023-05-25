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
		btScalar this_len = param->FOLLICLE_POS_ORIENT_LEN_VOL[f][6]; /*length already in mm*/
		btScalar this_mass = param->FOLLICLE_POS_ORIENT_LEN_VOL[f][7] /*volume already in mm^3*/ * param->fol_density;
		btTransform this_trans = createTransform(this_pos, this_ypr);
		Follicle* follicle = new Follicle(this, this_trans, param->fol_radius, this_len/2, this_mass, param->fol_damping, f);
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
	delete m_PM;
}

void MystacialPad::createLayer1() {
	std::cout << "Creating Layer1...";
	nTissueLayer1 = m_parameter->SPRING_HEX_MESH_IDX.size();

	for (int s = 0; s < nTissueLayer1; s++) {
		Follicle* fol1 = m_follicleArray[m_parameter->SPRING_HEX_MESH_IDX[s][0]];
		Follicle* fol2 = m_follicleArray[m_parameter->SPRING_HEX_MESH_IDX[s][1]];
		btTransform frameLayer1fol1 = createTransform(btVector3(m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->SPRING_HEX_MESH_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameLayer1fol2 = createTransform(btVector3(m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->SPRING_HEX_MESH_IDX[s][1]][6] / 2, 0., 0.));

		btScalar k_eq = m_parameter->k_layer1;
		btScalar k_this = k_eq / 2;
		Tissue* springLayer1 = new Tissue(m_sim, fol1->getBody(), fol2->getBody(), frameLayer1fol1, frameLayer1fol2, k_this, m_parameter->zeta_tissue);

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

		btScalar k_eq = m_parameter->k_layer2;
		btScalar k_this = k_eq / 2;
		Tissue* springLayer2 = new Tissue(m_sim, fol1->getBody(), fol2->getBody(), frameLayer2fol1, frameLayer2fol2, k_this, m_parameter->zeta_tissue);

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
		Tissue* tissueAnchor = new Tissue(m_sim, fol->getBody(), frameAnchor, m_parameter->k_anchor, m_parameter->zeta_tissue);	// this is a linear + torsional spring
																										
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
		IntrinsicSlingMuscle* muscle = new IntrinsicSlingMuscle(m_sim, folC->getBody(), folR->getBody(), frameC, frameR, m_parameter->f0_ISM, s /*userIndex*/);
		getWorld()->addConstraint(muscle->getConstraint(), true); // disable collision
		m_ISMArray.push_back(muscle);
	}
	// greek muscles
	for (int g = 0; g < m_parameter->INTRINSIC_SLING_MUSCLE_GREEK.size(); g++) {
		// constract nodes
		btTransform t = createTransform(btVector3(m_parameter->INTRINSIC_SLING_MUSCLE_GREEK[g][1],
			m_parameter->INTRINSIC_SLING_MUSCLE_GREEK[g][2], m_parameter->INTRINSIC_SLING_MUSCLE_GREEK[g][3]));
		btCollisionShape* s = new btSphereShape(0.1);
		btRigidBody* b = createDynamicBody(0 /*mass*/, t, s);
		m_ISM_nodes.push_back(b);
		getWorld()->addRigidBody(b, COL_EXT_MUS, extMusCollideWith);
		b->setActivationState(DISABLE_DEACTIVATION);

		// construct intrinsic muscle
		Follicle* folC = m_follicleArray[m_parameter->INTRINSIC_SLING_MUSCLE_GREEK[g][0]];
		btTransform frameC = createTransform(btVector3(-m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[g][0]][6] / 2 * 0.4, 0., 0.));
		IntrinsicSlingMuscle* muscle = new IntrinsicSlingMuscle(m_sim, folC->getBody(), b, frameC, createTransform(), m_parameter->f0_ISM);
		getWorld()->addConstraint(muscle->getConstraint(), true); // disable collision
		m_ISMArray.push_back(muscle);
		nISM++;
	}

	std::cout << "Done.\n";
}

void MystacialPad::createNasolabialis() {
	std::cout << "Creating extrinsic muscles: M.Nasolabialis ...";
	m_nasolabialis = new ExtrinsicMuscle(m_parameter->f0_nasolabialis, m_sim, m_parameter, m_follicleArray,
		m_parameter->NASOLABIALIS_NODE_POS, m_parameter->NASOLABIALIS_CONSTRUCTION_IDX, m_parameter->NASOLABIALIS_INSERTION_IDX, heightPlaceHolder);
	std::cout << "Done.\n";
}


void MystacialPad::createMaxillolabialis() {
	std::cout << "Creating extrinsic muscles: M.Maxillolabialis ...";
	m_maxillolabialis = new ExtrinsicMuscle(m_parameter->f0_maxillolabialis, m_sim, m_parameter, m_follicleArray,
		m_parameter->MAXILLOLABIALIS_NODE_POS, m_parameter->MAXILLOLABIALIS_CONSTRUCTION_IDX, m_parameter->MAXILLOLABIALIS_INSERTION_IDX, heightPlaceHolder);
	std::cout << "Done." << std::endl;
}

void MystacialPad::contractMuscle(Muscle mus, btScalar ratio) {
	switch (mus) {
	case Muscle::INTRINSIC:
		std::cout << "Contracting intrinsic muscle...\n";
		for (int s = 0; s < nISM; s++) {
			m_ISMArray[s]->contractTo(ratio);
		}
		break;
	case Muscle::N:
		std::cout << "Contracting nasolabialis (N)...\n";
		m_nasolabialis->contractTo(ratio);
		break;
	case Muscle::M:
		std::cout << "Contracting maxillolabialis (M)...\n";
		m_maxillolabialis->contractTo(ratio);
		break;
	case Muscle::NS:
		std::cout << "Contracting nasolabialis superficialis (NS)...\n";
		m_NS->contractTo(ratio);
		break;
	case Muscle::PMS:
		std::cout << "Contracting pars media superior (PMS)...\n";
		m_PMS->contractTo(ratio);
		break;
	case Muscle::PMI:
		std::cout << "Contracting pars media inferior (PMI)...\n";
		m_PMI->contractTo(ratio);
		break;
	case Muscle::PIP:
		std::cout << "Contracting pars interna produnda (PIP)...\n";
		m_PIP->contractTo(ratio);
		break;
	case Muscle::PM:
		std::cout << "Contracting pars maxilloris (PM)...\n";
		m_PM->contractTo(ratio);
		break;
	}
}

void MystacialPad::createNasolabialisSuperficialis() {
	std::cout << "Creating extrinsic muscles: M.Nasolabialis superficialis ...";
	m_NS = new ExtrinsicMuscle(m_parameter->f0_NS, m_sim, m_parameter, m_follicleArray,
		m_parameter->NASOLABIALIS_SUPERFICIALIS_NODE_POS, m_parameter->NASOLABIALIS_SUPERFICIALIS_CONSTRUCTION_IDX, m_parameter->NASOLABIALIS_SUPERFICIALIS_INSERTION_IDX, heightPlaceHolder,
		std::set<int>{ 0, 12, 13, 14, 15, 16 });
	std::cout << "Done.\n";
}

void MystacialPad::createParsMediaSuperior() {
	std::cout << "Creating extrinsci muscles: Pars media superior of M. Nasolabialis profundus...";
	m_PMS = new ExtrinsicMuscle(m_parameter->f0_PMS, m_sim, m_parameter, m_follicleArray, m_parameter->PARS_MEDIA_SUPERIOR_NODE_POS,
		m_parameter->PARS_MEDIA_SUPERIOR_CONSTRUCTION_IDX, m_parameter->PARS_MEDIA_SUPERIOR_INSERTION_IDX, m_parameter->PARS_MEDIA_SUPERIOR_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsMediaInferior() {
	std::cout << "Creating extrinsci muscles: Pars media inferior of M. Nasolabialis profundus...";
	m_PMI = new ExtrinsicMuscle(m_parameter->f0_PMI, m_sim, m_parameter, m_follicleArray, m_parameter->PARS_MEDIA_INFERIOR_NODE_POS,
		m_parameter->PARS_MEDIA_INFERIOR_CONSTRUCTION_IDX, m_parameter->PARS_MEDIA_INFERIOR_INSERTION_IDX, m_parameter->PARS_MEDIA_INFERIOR_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsInternaProfunda() {
	std::cout << "Creating extrinsci muscles: Pars interna profunda of M. Nasolabialis profundus...";
	m_PIP = new ExtrinsicMuscle(m_parameter->f0_PIP, m_sim, m_parameter, m_follicleArray, m_parameter->PARS_INTERNA_PROFUNDA_NODE_POS,
		m_parameter->PARS_INTERNA_PROFUNDA_CONSTRUCTION_IDX, m_parameter->PARS_INTERNA_PROFUNDA_INSERTION_IDX, m_parameter->PARS_INTERNA_PROFUNDA_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsMaxillaris() {
	std::cout << "Creating extrinsci muscles: Pars maxillaris (superficialis and profunda combined) of M. Nasolabialis profundus...";
	m_PM = new ExtrinsicMuscle(m_parameter->f0_PM, m_sim, m_parameter, m_follicleArray, m_parameter->PARS_MAXILLARIS_NODE_POS,
		m_parameter->PARS_MAXILLARIS_CONSTRUCTION_IDX, m_parameter->PARS_MAXILLARIS_INSERTION_IDX, m_parameter->PARS_MAXILLARIS_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::update(btScalar dt) {
	PROFILE_FUNCTION();
	static btScalar timeElapsed = 0.0f;
	timeElapsed += dt;
	bool fiberQueryFlag = false;
	if (timeElapsed >= m_parameter->inverse_fiber_query_rate) {
		fiberQueryFlag = true;
		timeElapsed -= m_parameter->inverse_fiber_query_rate;
	}

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

	if (fiberQueryFlag) {
		for (int i = 0; i < nISM; i++) {
			m_ISMArray[i]->update();
		}
	}

	if (m_nasolabialis)		m_nasolabialis->update(fiberQueryFlag);
	if (m_maxillolabialis)	m_maxillolabialis->update(fiberQueryFlag);
	if (m_NS)				m_NS->update(fiberQueryFlag);
	if (m_PMS)				m_PMS->update(fiberQueryFlag);
	if (m_PMI)				m_PMI->update(fiberQueryFlag);
	if (m_PIP)				m_PIP->update(fiberQueryFlag);
	if (m_PM)				m_PM->update(fiberQueryFlag);

	
}

void MystacialPad::readOutput(std::vector<std::vector<std::vector<btScalar>>>& output) {
	// output all follicle top/bottom pos
	for (int i = 0; i < nFollicle; i++) {
		btVector3 pos_top = m_follicleArray[i]->getTopLocation();
		btVector3 pos_bot = m_follicleArray[i]->getBotLocation();
		std::vector<btScalar> posVec{ pos_top[0], pos_top[1], pos_top[2],
									  pos_bot[0], pos_bot[1], pos_bot[2] };

		output[i].push_back(posVec);
	}
}

void MystacialPad::debugDraw() {

	//for (int i = 0; i < m_layer1.size(); i++) {
	//	m_layer1[i]->debugDraw(btVector3(1., 0., 0.), false);
	//}
	//for (int i = 0; i < m_layer2.size(); i++) {
	//	m_layer2[i]->debugDraw(btVector3(1., 0., 0.), false);
	//}
	//for (int i = 0; i < nTissueAnchor; i++) {
	//	m_anchor[i]->debugDraw(RED, true);
	//}
	//for (int i = 0; i < nISM; i++) {
	//	m_ISMArray[i]->debugDraw(RED, false);
	//}

	if (m_nasolabialis) {
		m_nasolabialis->debugDraw(GREEN);
	}
	if (m_maxillolabialis) {
		m_maxillolabialis->debugDraw(BLUE);
	}
	if (m_NS) {
		m_NS->debugDraw(BLUE);
	}
	if (m_PMS) {
		m_PMS->debugDraw(ORANGE);
	}
	if (m_PIP) {
		m_PIP->debugDraw(ORANGE);
	}
	if (m_PMI) {
		m_PMI->debugDraw(YELLOW);
	}
	if (m_PM) {
		m_PM->debugDraw(YELLOW);
	}
}

int MystacialPad::getNumFollicles() const {
	return nFollicle;
}

Follicle* MystacialPad::getFollicleByIndex(int idx) {
	return m_follicleArray[idx];
}




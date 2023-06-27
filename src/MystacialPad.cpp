#include "my_pch.h"
#include "MystacialPad.h"

#include "Utility.h"
#include "Parameter.h"
#include "Follicle.h"
#include "Tissue.h"
#include "Layer.h"
#include "IntrinsicMuscle.h"
#include "ExtrinsicMuscle.h"


MystacialPad::MystacialPad(Simulation* sim, Parameter* param)
	: m_sim(sim), m_parameter(param)
	, nFollicle(0), nISM(0)
	, m_nasolabialis(nullptr), m_maxillolabialis(nullptr), m_NS(nullptr)
	, m_PMS(nullptr), m_PMI(nullptr), m_PIP(nullptr), m_PM(nullptr), m_Hamiltonian() {

	m_layer1 = std::make_unique<Layer>(sim, param, this);
	m_layer2 = std::make_unique<Layer>(sim, param, this);

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
		std::unique_ptr<Follicle> follicle = std::make_unique<Follicle>(this, this_trans, param->fol_radius, this_len/2, this_mass, param->fol_damping, f);
		follicle->setUserPointer(follicle->getInfo());
		m_follicleArray.push_back(std::move(follicle));
	}
	std::cout << "Done.\n";
}

void MystacialPad::createLayer1() {
	std::cout << "Creating Layer1...";

	m_layer1->initEdges(true);
	m_layer1->initAnchors(true);

	switch (m_parameter->getBendingModel()) {
	case BENDING_MODEL::SPRING: {
		m_layer1->initBendings(true);
		break;
	}
	case BENDING_MODEL::DIHEDRAL_ANGLE: {
		// only works for reduced model now
		ensure(m_parameter->getArrayModel() == MODEL::REDUCED);
		m_layer1->initDihedralPairs(true);
		break;
	}
	}

	if (m_parameter->getBendingModel() == BENDING_MODEL::SPRING) {
		

	}
	std::cout << "Done.\n";
}

void MystacialPad::createLayer2() {
	std::cout << "Creating Layer2...";

	m_layer2->initEdges(false);

	switch (m_parameter->getBendingModel()) {
	case BENDING_MODEL::SPRING: {
		m_layer2->initBendings(false);
		break;
	}
	case BENDING_MODEL::DIHEDRAL_ANGLE: {
		// only works for reduced model now
		ensure(m_parameter->getArrayModel() == MODEL::REDUCED);
		m_layer2->initDihedralPairs(false);
		break;
	}
	}
	std::cout << "Done.\n";
}


void MystacialPad::createIntrinsicSlingMuscle() {
	if (!checkCreateMuscleFlag(MUSCLE::ISM)) {
		return;
	}

	std::cout << "Creating intrinsic sling muscles...";
	nISM = m_parameter->INTRINSIC_SLING_MUSCLE_IDX.size();
	for (int s = 0; s < nISM; s++) {
		auto& folC = m_follicleArray[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[s][0]];
		auto& folR = m_follicleArray[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[s][1]];
		btTransform frameC = createTransform(btVector3(m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameR = createTransform(btVector3(-m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[s][1]][6] / 2 * 0.4, 0., 0.)); // 70% of rostral member
		std::unique_ptr<IntrinsicSlingMuscle> muscle = std::make_unique<IntrinsicSlingMuscle>(m_sim, folC->getBody(), folR->getBody(), frameC, frameR, m_parameter->f0_ISM, s /*userIndex*/);
		getWorld()->addConstraint(muscle->getConstraint(), true); // disable collision
		m_ISMArray.push_back(std::move(muscle));
	}
	// greek muscles
	for (int g = 0; g < m_parameter->INTRINSIC_SLING_MUSCLE_GREEK.size(); g++) {
		// constract nodes
		btTransform t = createTransform(btVector3(m_parameter->INTRINSIC_SLING_MUSCLE_GREEK[g][1],
			m_parameter->INTRINSIC_SLING_MUSCLE_GREEK[g][2], m_parameter->INTRINSIC_SLING_MUSCLE_GREEK[g][3]));
		btCollisionShape* s = new btSphereShape(0.1);
		btRigidBody* b = createDynamicBody(0 /*mass*/, t, s);
		getWorld()->addRigidBody(b, COL_EXT_MUS, extMusCollideWith);
		b->setActivationState(DISABLE_DEACTIVATION);

		// construct intrinsic muscle
		auto& folC = m_follicleArray[m_parameter->INTRINSIC_SLING_MUSCLE_GREEK[g][0]];
		btTransform frameC = createTransform(btVector3(-m_parameter->FOLLICLE_POS_ORIENT_LEN_VOL[m_parameter->INTRINSIC_SLING_MUSCLE_IDX[g][0]][6] / 2 * 0.4, 0., 0.));
		std::unique_ptr<IntrinsicSlingMuscle> muscle = std::make_unique<IntrinsicSlingMuscle>(m_sim, folC->getBody(), b, frameC, createTransform(), m_parameter->f0_ISM);
		getWorld()->addConstraint(muscle->getConstraint(), true); // disable collision
		m_ISMArray.push_back(std::move(muscle));
		nISM++;
	}

	std::cout << "Done.\n";
}

void MystacialPad::createNasolabialis() {
	if (!checkCreateMuscleFlag(MUSCLE::N)) {
		return;
	}
	std::cout << "Creating extrinsic muscles: M.Nasolabialis ...";
	m_nasolabialis = std::make_unique<ExtrinsicMuscle>(m_parameter->f0_nasolabialis, m_sim, m_parameter, this,
		m_parameter->NASOLABIALIS_NODE_POS, m_parameter->NASOLABIALIS_CONSTRUCTION_IDX, m_parameter->NASOLABIALIS_INSERTION_IDX, heightPlaceHolder);
	std::cout << "Done.\n";
}


void MystacialPad::createMaxillolabialis() {
	if (!checkCreateMuscleFlag(MUSCLE::M)) {
		return;
	}
	std::cout << "Creating extrinsic muscles: M.Maxillolabialis ...";
	m_maxillolabialis = std::make_unique<ExtrinsicMuscle>(m_parameter->f0_maxillolabialis, m_sim, m_parameter, this,
		m_parameter->MAXILLOLABIALIS_NODE_POS, m_parameter->MAXILLOLABIALIS_CONSTRUCTION_IDX, m_parameter->MAXILLOLABIALIS_INSERTION_IDX, heightPlaceHolder);
	std::cout << "Done." << std::endl;
}

void MystacialPad::contractMuscle(MUSCLE mus, btScalar ratio) {
	switch (m_parameter->FlagContractMuscle & mus) {
	case MUSCLE::ISM:
		std::cout << "Contracting intrinsic muscle...\n";
		for (int s = 0; s < nISM; s++) {
			m_ISMArray[s]->contractTo(ratio);
		}
		break;
	case MUSCLE::N:
		std::cout << "Contracting nasolabialis (N)...\n";
		m_nasolabialis->contractTo(ratio);
		break;
	case MUSCLE::M:
		std::cout << "Contracting maxillolabialis (M)...\n";
		m_maxillolabialis->contractTo(ratio);
		break;
	case MUSCLE::NS:
		std::cout << "Contracting nasolabialis superficialis (NS)...\n";
		m_NS->contractTo(ratio);
		break;
	case MUSCLE::PMS:
		std::cout << "Contracting pars media superior (PMS)...\n";
		m_PMS->contractTo(ratio);
		break;
	case MUSCLE::PMI:
		std::cout << "Contracting pars media inferior (PMI)...\n";
		m_PMI->contractTo(ratio);
		break;
	case MUSCLE::PIP:
		std::cout << "Contracting pars interna produnda (PIP)...\n";
		m_PIP->contractTo(ratio);
		break;
	case MUSCLE::PM:
		std::cout << "Contracting pars maxilloris (PM)...\n";
		m_PM->contractTo(ratio);
		break;
	}
}

void MystacialPad::createNasolabialisSuperficialis() {
	if (!checkCreateMuscleFlag(MUSCLE::NS)) {
		return;
	}
	std::cout << "Creating extrinsic muscles: M.Nasolabialis superficialis ...";
	m_NS = std::make_unique<ExtrinsicMuscle>(m_parameter->f0_NS, m_sim, m_parameter, this,
		m_parameter->NASOLABIALIS_SUPERFICIALIS_NODE_POS, m_parameter->NASOLABIALIS_SUPERFICIALIS_CONSTRUCTION_IDX, m_parameter->NASOLABIALIS_SUPERFICIALIS_INSERTION_IDX, heightPlaceHolder,
		std::set<int>{ 0, 12, 13, 14, 15, 16 });
	std::cout << "Done.\n";
}

void MystacialPad::createParsMediaSuperior() {
	if (!checkCreateMuscleFlag(MUSCLE::PMS)) {
		return;
	}
	std::cout << "Creating extrinsci muscles: Pars media superior of M. Nasolabialis profundus...";
	m_PMS = std::make_unique<ExtrinsicMuscle>(m_parameter->f0_PMS, m_sim, m_parameter, this, m_parameter->PARS_MEDIA_SUPERIOR_NODE_POS,
		m_parameter->PARS_MEDIA_SUPERIOR_CONSTRUCTION_IDX, m_parameter->PARS_MEDIA_SUPERIOR_INSERTION_IDX, m_parameter->PARS_MEDIA_SUPERIOR_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsMediaInferior() {
	if (!checkCreateMuscleFlag(MUSCLE::PMI)) {
		return;
	}
	std::cout << "Creating extrinsci muscles: Pars media inferior of M. Nasolabialis profundus...";
	m_PMI = std::make_unique<ExtrinsicMuscle>(m_parameter->f0_PMI, m_sim, m_parameter, this, m_parameter->PARS_MEDIA_INFERIOR_NODE_POS,
		m_parameter->PARS_MEDIA_INFERIOR_CONSTRUCTION_IDX, m_parameter->PARS_MEDIA_INFERIOR_INSERTION_IDX, m_parameter->PARS_MEDIA_INFERIOR_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsInternaProfunda() {
	if (!checkCreateMuscleFlag(MUSCLE::PIP)) {
		return;
	}
	std::cout << "Creating extrinsci muscles: Pars interna profunda of M. Nasolabialis profundus...";
	m_PIP = std::make_unique<ExtrinsicMuscle>(m_parameter->f0_PIP, m_sim, m_parameter, this, m_parameter->PARS_INTERNA_PROFUNDA_NODE_POS,
		m_parameter->PARS_INTERNA_PROFUNDA_CONSTRUCTION_IDX, m_parameter->PARS_INTERNA_PROFUNDA_INSERTION_IDX, m_parameter->PARS_INTERNA_PROFUNDA_INSERTION_HEIGHT);
	std::cout << "Done.\n";
}

void MystacialPad::createParsMaxillaris() {
	if (!checkCreateMuscleFlag(MUSCLE::PM)) {
		return;
	}
	std::cout << "Creating extrinsci muscles: Pars maxillaris (superficialis and profunda combined) of M. Nasolabialis profundus...";
	m_PM = std::make_unique<ExtrinsicMuscle>(m_parameter->f0_PM, m_sim, m_parameter, this, m_parameter->PARS_MAXILLARIS_NODE_POS,
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

	m_Hamiltonian = 0;
	for (int i = 0; i < nFollicle; i++) {
		m_follicleArray[i]->update();
		m_Hamiltonian += m_follicleArray[i]->getHamiltonian();
	}
	// only linear springs need update
	// torsional springs don't
	if (m_layer1) {
		m_layer1->update();
		m_Hamiltonian += m_layer1->getHamiltonian();
	}
	if (m_layer2) {
		m_layer2->update();
		m_Hamiltonian += m_layer2->getHamiltonian();
	}

	
	for (int i = 0; i < nISM; i++) {
		if (fiberQueryFlag) {
			m_ISMArray[i]->update();
		}
		m_Hamiltonian += m_ISMArray[i]->getHamiltonian();
	}
		
	if (m_nasolabialis) { 
		m_nasolabialis->update(fiberQueryFlag);		
		m_Hamiltonian += m_nasolabialis->getHamiltonian(); 
	}
	if (m_maxillolabialis) { 
		m_maxillolabialis->update(fiberQueryFlag);	
		m_Hamiltonian += m_maxillolabialis->getHamiltonian(); 
	}
	if (m_NS) { 
		m_NS->update(fiberQueryFlag);					
		m_Hamiltonian += m_NS->getHamiltonian(); 
	}
	if (m_PMS) { 
		m_PMS->update(fiberQueryFlag);				
		m_Hamiltonian += m_PMS->getHamiltonian(); 
	}
	if (m_PMI) { 
		m_PMI->update(fiberQueryFlag);				
		m_Hamiltonian += m_PMI->getHamiltonian(); 
	}
	if (m_PIP) { 
		m_PIP->update(fiberQueryFlag);				
		m_Hamiltonian += m_PIP->getHamiltonian(); 
	}
	if (m_PM) { 
		m_PM->update(fiberQueryFlag);					
		m_Hamiltonian += m_PM->getHamiltonian(); 
	}

	
}

void MystacialPad::bufferFolPos(std::vector<std::vector<std::vector<btScalar>>>& output) {
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
	if (m_layer1) {
		m_layer1->debugDraw(btVector3(1., 0., 0.), true);
	}
	if (m_layer2) {
		m_layer2->debugDraw(btVector3(1., 0., 0.), true);
	}

	for (int i = 0; i < nISM; i++) {
		m_ISMArray[i]->debugDraw(RED, false);
	}

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

std::unique_ptr<Follicle>& MystacialPad::getFollicleByIndex(int idx) {
	return m_follicleArray[idx];
}

bool MystacialPad::checkCreateMuscleFlag(MUSCLE mus) const {
	return (m_parameter->FlagCreateMuscles & mus) == mus;
}
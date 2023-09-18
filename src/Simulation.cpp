#include "my_pch.h"
#include "Simulation.h"

#include "Parameter.h"
#include "Instrumentor.h"
#include "Utility.h"
#include "MystacialPad.h"
#include "IntrinsicMuscle.h"
#include "ExtrinsicMuscle.h"
#include "Tissue.h"
#include "Follicle.h"
#include "myGeneric6DofMuscleConstraint.h"

#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "CommonInterfaces/CommonParameterInterface.h"

#define BIT(x) (1<<(x))


Simulation::~Simulation() {
}


void Simulation::stepSimulation(float deltaTime) {
	//PROFILE_FUNCTION();
	auto start = std::chrono::high_resolution_clock::now();
	m_time += deltaTime; 								// increase time
	m_step += 1;										// increase step

	if (m_mystacialPad && (param->m_time_stop == 0 || m_time < param->m_time_stop)) {
		// protraction/retraction (can overlap)
		{
			calculateContractionPhase();
			// phase 1: ISM protractors
			m_mystacialPad->contractMuscle(MUSCLE::ISM, phase1_ratio);
			m_mystacialPad->contractMuscle(MUSCLE::NS,  phase1_ratio);
			m_mystacialPad->contractMuscle(MUSCLE::POO,  phase1_ratio);
			m_mystacialPad->contractMuscle(MUSCLE::PMS, phase1_ratio);
			m_mystacialPad->contractMuscle(MUSCLE::PMI, phase1_ratio);
			// phase 2: extrinsic retractors, including N, M, PIP, PM
			m_mystacialPad->contractMuscle(MUSCLE::N,   phase2_ratio);
			m_mystacialPad->contractMuscle(MUSCLE::M,   phase2_ratio);
			m_mystacialPad->contractMuscle(MUSCLE::PIP, phase2_ratio);
			m_mystacialPad->contractMuscle(MUSCLE::PM,  phase2_ratio);
		}

		// update physics constraint such as spring equilibirum point or fiber forces
		m_mystacialPad->preUpdate(deltaTime);

		// last step: step simulation
		{
			PROFILE_SCOPE("m_dynamicsWorld->StepSimulation()");
			m_dynamicsWorld->stepSimulation(deltaTime,	// rendering time step
				param->m_num_internal_step * 100,		// max sub step
				param->m_internal_time_step);			// fixed simulation sub time step
		}

		// update other attributes after step simulation, such as follicle top/bot location
		m_mystacialPad->postUpdate();

		//// post step simulation: apply additional damping
		{
			applyAdditionalDamping();
		}

		//// collision listener
		updateCollisionListener();

		// set up output options
		if (param->OUTPUT) {
			m_mystacialPad->bufferFolPos(output_fol_pos);
			S_dumpster::Get().Update();
			//S_dumpster::Get().hamiltonian.push_back(m_mystacialPad->getHamiltonian());
		}

		// debug draw
		if (param->DEBUG) {
			PROFILE_SCOPE("DebugDraw()");
			m_mystacialPad->debugDraw();
			m_dynamicsWorld->debugDrawWorld();

			draw_text.clear();
			draw_text += m_mystacialPad->getDrawText() + "\n";
		}

	}
	else {
		// timeout -> set exit flag
		exitSim = true;
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	m_time_elapsed += duration.count() / 1000.f;
	auto factor = m_time_elapsed / m_time;
	auto time_remaining = (int)((param->m_time_stop - m_time) * (factor));
}


void Simulation::initPhysics() {
	preInitPhysics();

	// Initializing physics world
	// create and contract muscles
	////////////////////////////////////////////////////////////////////////////////
	read_csv_float(param->dir_follicle_pos_orient_len_vol, param->FOLLICLE_POS_ORIENT_LEN_VOL);
	m_mystacialPad = new MystacialPad(this, param);
	

	// layers
	read_csv_int(param->dir_spring_hex_mesh_idx, param->SPRING_HEX_MESH_IDX);
	read_csv_int(param->dir_spring_bending_idx, param->SPRING_BENDING_IDX);
	m_mystacialPad->createLayer1();
	m_mystacialPad->createLayer2();

	//// intrinsic sling muscles
	read_csv_int(param->dir_intrinsic_sling_muscle_idx, param->INTRINSIC_SLING_MUSCLE_IDX);
	read_csv_float(param->dir_intrinsic_sling_muscle_greek, param->INTRINSIC_SLING_MUSCLE_GREEK);
	m_mystacialPad->createIntrinsicSlingMuscle();

	
	// extrinsic: nasolabialis muscle
	read_csv_float(param->dir_nasolabialis_node_pos, param->NASOLABIALIS_NODE_POS);
	read_csv_int(param->dir_nasolabialis_construction_idx, param->NASOLABIALIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_nasolabialis_insertion_idx, param->NASOLABIALIS_INSERTION_IDX);
	m_mystacialPad->createNasolabialis();


	// extrinsic: maxillolabialis muscle
	read_csv_float(param->dir_maxillolabialis_node_pos, param->MAXILLOLABIALIS_NODE_POS);
	read_csv_int(param->dir_maxillolabialis_construction_idx, param->MAXILLOLABIALIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_maxillolabialis_insertion_idx, param->MAXILLOLABIALIS_INSERTION_IDX);
	m_mystacialPad->createMaxillolabialis();


	//// extrinsic: nasolabialis superficialis
	read_csv_float(param->dir_nasolabialis_superficialis_node_pos, param->NASOLABIALIS_SUPERFICIALIS_NODE_POS);
	read_csv_int(param->dir_nasolabialis_superficialis_construction_idx, param->NASOLABIALIS_SUPERFICIALIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_nasolabialis_superficialis_insertion_idx, param->NASOLABIALIS_SUPERFICIALIS_INSERTION_IDX);
	m_mystacialPad->createNasolabialisSuperficialis();

	//// extrinsic: pars orbicularis oris
	read_csv_float(param->dir_pars_orbicularis_oris_node_pos, param->PARS_ORBICULARIS_ORIS_NODE_POS);
	read_csv_int(param->dir_pars_orbicularis_oris_construction_idx, param->PARS_ORBICULARIS_ORIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_pars_orbicularis_oris_insertion_idx, param->PARS_ORBICULARIS_ORIS_INSERTION_IDX);
	m_mystacialPad->createParsOrbicularisOris();


	//// extrinsic: pars media superior of M. Nasolabialis profundus
	// corium
	read_csv_float(param->dir_pars_media_superior_node_pos, param->PARS_MEDIA_SUPERIOR_NODE_POS);
	read_csv_int(param->dir_pars_media_superior_construction_idx, param->PARS_MEDIA_SUPERIOR_CONSTRUCTION_IDX);
	read_csv_int(param->dir_pars_media_superior_insertion_idx, param->PARS_MEDIA_SUPERIOR_INSERTION_IDX);
	read_csv_float(param->dir_pars_media_superior_insertion_height, param->PARS_MEDIA_SUPERIOR_INSERTION_HEIGHT);
	m_mystacialPad->createParsMediaSuperior();

	//// extrinsic: pars interna profunda of M. Nasolabialis profundus
	// subcapsular
	read_csv_float(param->dir_pars_interna_profunda_node_pos, param->PARS_INTERNA_PROFUNDA_NODE_POS);
	read_csv_int(param->dir_pars_interna_profunda_construction_idx, param->PARS_INTERNA_PROFUNDA_CONSTRUCTION_IDX);
	read_csv_int(param->dir_pars_interna_profunda_insertion_idx, param->PARS_INTERNA_PROFUNDA_INSERTION_IDX);
	read_csv_float(param->dir_pars_interna_profunda_insertion_height, param->PARS_INTERNA_PROFUNDA_INSERTION_HEIGHT);
	m_mystacialPad->createParsInternaProfunda();


	//// extrinsic: pars media inferior of M. Nasolabialis profundus
	// corium
	read_csv_float(param->dir_pars_media_inferior_node_pos, param->PARS_MEDIA_INFERIOR_NODE_POS);
	read_csv_int(param->dir_pars_media_inferior_construction_idx, param->PARS_MEDIA_INFERIOR_CONSTRUCTION_IDX);
	read_csv_int(param->dir_pars_media_inferior_insertion_idx, param->PARS_MEDIA_INFERIOR_INSERTION_IDX);
	read_csv_float(param->dir_pars_media_inferior_insertion_height, param->PARS_MEDIA_INFERIOR_INSERTION_HEIGHT);
	m_mystacialPad->createParsMediaInferior();

	//// extrinsic: pars maxillaris & profunda of M. Nasolabialis profundus
	// subcapsular
	read_csv_float(param->dir_pars_maxillaris_node_pos, param->PARS_MAXILLARIS_NODE_POS);
	read_csv_int(param->dir_pars_maxillaris_construction_idx, param->PARS_MAXILLARIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_pars_maxillaris_insertion_idx, param->PARS_MAXILLARIS_INSERTION_IDX);
	read_csv_float(param->dir_pars_maxillaris_insertion_height, param->PARS_MAXILLARIS_INSERTION_HEIGHT);
	m_mystacialPad->createParsMaxillaris();

	postInitPhysics();
}

void Simulation::initPhysics_reduced() {
	PROFILE_FUNCTION();
	preInitPhysics();

	// Initializing physics world
	// create and contract muscles
	////////////////////////////////////////////////////////////////////////////////
	read_csv_float(param->dir_follicle_pos_orient_len_vol_reduced, param->FOLLICLE_POS_ORIENT_LEN_VOL);
	read_csv_int(param->dir_spring_bending_idx_reduced, param->SPRING_BENDING_IDX);
	m_mystacialPad = new MystacialPad(this, param);

	read_csv_int(param->dir_spring_hex_mesh_idx_reduced, param->SPRING_HEX_MESH_IDX);
	m_mystacialPad->createLayer1();
	m_mystacialPad->createLayer2();

	//// intrinsic sling muscles
	read_csv_int(param->dir_intrinsic_sling_muscle_idx_reduced, param->INTRINSIC_SLING_MUSCLE_IDX);
	read_csv_float(param->dir_intrinsic_sling_muscle_greek_reduced, param->INTRINSIC_SLING_MUSCLE_GREEK);
	m_mystacialPad->createIntrinsicSlingMuscle();

	// extrinsic: nasolabialis muscle
	read_csv_float(param->dir_nasolabialis_node_pos_reduced, param->NASOLABIALIS_NODE_POS);
	read_csv_int(param->dir_nasolabialis_construction_idx_reduced, param->NASOLABIALIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_nasolabialis_insertion_idx_reduced, param->NASOLABIALIS_INSERTION_IDX);
	m_mystacialPad->createNasolabialis();

	// extrinsic: maxillolabialis muscle
	read_csv_float(param->dir_maxillolabialis_node_pos_reduced, param->MAXILLOLABIALIS_NODE_POS);
	read_csv_int(param->dir_maxillolabialis_construction_idx_reduced, param->MAXILLOLABIALIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_maxillolabialis_insertion_idx_reduced, param->MAXILLOLABIALIS_INSERTION_IDX);
	m_mystacialPad->createMaxillolabialis();

	//// extrinsic: pars media superior of M. Nasolabialis profundus
	// corium
	read_csv_float(param->dir_pars_media_superior_node_pos_reduced, param->PARS_MEDIA_SUPERIOR_NODE_POS);
	read_csv_int(param->dir_pars_media_superior_construction_idx_reduced, param->PARS_MEDIA_SUPERIOR_CONSTRUCTION_IDX);
	read_csv_int(param->dir_pars_media_superior_insertion_idx_reduced, param->PARS_MEDIA_SUPERIOR_INSERTION_IDX);
	read_csv_float(param->dir_pars_media_superior_insertion_height_reduced, param->PARS_MEDIA_SUPERIOR_INSERTION_HEIGHT);
	m_mystacialPad->createParsMediaSuperior();

	//// extrinsic: pars interna profunda of M. Nasolabialis profundus
	// subcapsular
	read_csv_float(param->dir_pars_interna_profunda_node_pos_reduced, param->PARS_INTERNA_PROFUNDA_NODE_POS);
	read_csv_int(param->dir_pars_interna_profunda_construction_idx_reduced, param->PARS_INTERNA_PROFUNDA_CONSTRUCTION_IDX);
	read_csv_int(param->dir_pars_interna_profunda_insertion_idx_reduced, param->PARS_INTERNA_PROFUNDA_INSERTION_IDX);
	read_csv_float(param->dir_pars_interna_profunda_insertion_height_reduced, param->PARS_INTERNA_PROFUNDA_INSERTION_HEIGHT);
	m_mystacialPad->createParsInternaProfunda();

	//// extrinsic: pars media inferior of M. Nasolabialis profundus
	// corium
	read_csv_float(param->dir_pars_media_inferior_node_pos_reduced, param->PARS_MEDIA_INFERIOR_NODE_POS);
	read_csv_int(param->dir_pars_media_inferior_construction_idx_reduced, param->PARS_MEDIA_INFERIOR_CONSTRUCTION_IDX);
	read_csv_int(param->dir_pars_media_inferior_insertion_idx_reduced, param->PARS_MEDIA_INFERIOR_INSERTION_IDX);
	read_csv_float(param->dir_pars_media_inferior_insertion_height_reduced, param->PARS_MEDIA_INFERIOR_INSERTION_HEIGHT);
	m_mystacialPad->createParsMediaInferior();

	//// extrinsic: pars maxillaris & profunda of M. Nasolabialis profundus
	// subcapsular
	read_csv_float(param->dir_pars_maxillaris_node_pos_reduced, param->PARS_MAXILLARIS_NODE_POS);
	read_csv_int(param->dir_pars_maxillaris_construction_idx_reduced, param->PARS_MAXILLARIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_pars_maxillaris_insertion_idx_reduced, param->PARS_MAXILLARIS_INSERTION_IDX);
	read_csv_float(param->dir_pars_maxillaris_insertion_height_reduced, param->PARS_MAXILLARIS_INSERTION_HEIGHT);
	m_mystacialPad->createParsMaxillaris();

	postInitPhysics();
};


void Simulation::resetCamera() {
	m_guiHelper->resetCamera(param->camDist, param->camYaw, param->camPitch, 
		 param->camPos[0], param->camPos[1], param->camPos[2]);
}


void Simulation::initPhysics_test() {
	preInitPhysics();

	// add boxes
	btCollisionShape* boxShape = new btBoxShape(btVector3(1, 1, 1));
	m_collisionShapes.push_back(boxShape);

	box1 = createDynamicBody(1, createTransform(btVector3(5, 0, 0), btVector3(PI / 6, 0, 0)), boxShape);
	m_dynamicsWorld->addRigidBody(box1, COL_FOLLICLE, follicleCollideWith);
	box1->setActivationState(DISABLE_DEACTIVATION);

	box2 = createDynamicBody(1, createTransform(btVector3(-5, 0, 0)), boxShape);
	m_dynamicsWorld->addRigidBody(box2, COL_FOLLICLE, follicleCollideWith);
	box2->setActivationState(DISABLE_DEACTIVATION);

	

	//// add constraints
	btGeneric6DofSpring2Constraint* torsional_constraint = new btGeneric6DofSpring2Constraint(*box1, *box2, createTransform(), createTransform());
	torsional_constraint->setLinearLowerLimit(btVector3(-10, 1, 1));	// need to set lower > higher to free the dofs
	torsional_constraint->setLinearUpperLimit(btVector3(-10, 0, 0));
	torsional_constraint->setAngularLowerLimit(btVector3(1, 1, 1));
	torsional_constraint->setAngularUpperLimit(btVector3(0, 0, 0));
	torsional_constraint->setEquilibriumPoint(1);
	for (int i = 1; i < 3; i++) {
		torsional_constraint->enableSpring(i, true);
		torsional_constraint->setStiffness(i, 1000000);
		torsional_constraint->setDamping(i, 100);
		torsional_constraint->setEquilibriumPoint(i);
	}
	//for (int i = 3; i < 6; i++) {
	//	torsional_constraint->enableSpring(i, true);
	//	torsional_constraint->setStiffness(i, 1000.f);
	//	torsional_constraint->setDamping(i, 0.f);
	//	torsional_constraint->setEquilibriumPoint(i);
	//}
	m_dynamicsWorld->addConstraint(torsional_constraint, true);

	//box1->setAngularVelocity(btVector3(0, 0, 2));

	postInitPhysics();
}

void Simulation::stepSimulation_test(float deltaTime) {
	auto start = std::chrono::high_resolution_clock::now();

	static int frame = 0;
	frame++;
	static int updates = 0;
	static btScalar timeElapsed = 0.0f;
	timeElapsed += deltaTime;
	bool updateFiber = false;
	if (timeElapsed >= (1.0f / 30.0f)) {
		updateFiber = true;
		timeElapsed -= (1.0f / 30.0f);
	}

	if (param->m_time_stop == 0 || m_time <= param->m_time_stop) {
		// update everything here:
		btVector3 force(1, 0, 0);
		//box1->applyCentralForce(force);
		//box1->applyForce(force, btVector3(0, 0, 0.5));
		//box1->applyTorque(btVector3(0, 0, 1));
		//box1->applyCentralImpulse(force * param->m_time_step);
		//box1->applyImpulse(force * param->m_time_step, btVector3(0, 0, 0.5));

		btVector3 box1_LinearVelocity = box1->getLinearVelocity();
		btVector3 box1_AngularVelocity = box1->getAngularVelocity();
		btVector3 box1_PointVelocity = box1_LinearVelocity +
			box1_AngularVelocity.cross(box1->getCenterOfMassTransform().getBasis() * btVector3(1, 1, 1));

		//if (t1) t1->update();
		//if (t2) t2->update();
		//if (f) {
		//	//// contraction/retraction
		//	//int every_steps = param->getFPS() / param->contract_frequency / 2.0f;
		//	//static btScalar range = 0.6; // contract_range;
		//	//if ((m_step - 1) % every_steps == 0) {
		//	//	range = -range;
		//	//	f->contractTo(1.0 - param->contract_range / 2 + range / 2);
		//	//}
		//	f->contractTo(0.8);
		//	if (updateFiber) {
		//		f->update();
		//		updates++;
		//	}
		//}
		//else {
		//	btVector3 fVal = { 5.0f, 0.0f, 0.0f };
		//	box1->applyCentralForce(-fVal);
		//	//box2->applyCentralForce(fVal);
		//}


		btVector3 box1pos = box1->getCenterOfMassPosition();
		S_dumpster::Get().test_info[0].push_back(box1pos[0]);

		// last step: step simulation
		m_dynamicsWorld->stepSimulation(deltaTime,	// rendering time step
			param->m_num_internal_step * 100,		// max sub step
			param->m_internal_time_step);			// fixed simulation sub time step

		if (param->DEBUG) {
			m_dynamicsWorld->debugDrawWorld();
		}
	}
	else {
		char buffer[32];
		sprintf(buffer, "test_%d.csv", param->m_fps);
		//write_csv_float("../output", buffer, S_dumpster::Get().test_info);

		// timeout -> set exit flag
		exitSim = true;
	}

	m_time += param->m_time_step; 						// increase time
	m_step += 1;										// increase step

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	m_time_elapsed += duration.count() / 1000.f;
	auto factor = m_time_elapsed / m_time;
	auto time_remaining = (int)((param->m_time_stop - m_time) * (factor));


}


void Simulation::preInitPhysics() {
	std::cout << "Physics engine initiating...." << std::endl;
	// set visual axis
	m_guiHelper->setUpAxis(2);

	// create empty dynamics world[0]
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	// broadphase algorithm: dynamic bounding volume tree (DBVT)
	m_broadphase = new btDbvtBroadphase();

	// select solver
	std::cout << "Using btSequentialImpulseConstraintSolver..." << std::endl;
	m_solver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

	// set number of iterations for contact solver
	/* this is for contact solver, has very limited effect on dynamics solver */
	m_dynamicsWorld->getSolverInfo().m_timeStep = 1 / 10;	// btScalar(1.0f / param->getFPS());
	m_dynamicsWorld->getSolverInfo().m_numIterations = 20;  //param->m_num_internal_step;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD |
		SOLVER_USE_WARMSTARTING |
		SOLVER_RANDMIZE_ORDER |
		0;
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
	m_dynamicsWorld->getSolverInfo().m_erp = 0.8f;

	// set gravity
	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));

	// setup debug drawer
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	if (m_dynamicsWorld->getDebugDrawer()) {
		if (param->DEBUG == 0 || param->DEBUG == 1) { // 
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_NoDebug);
		}
		else if (param->DEBUG == 2) { // wire frame (collision bounds)
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		}
		else if (param->DEBUG == 3) { // Axis aligned bound box
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawAabb);
		}
	}
}

void Simulation::postInitPhysics() {
	int total_frame = (int)param->m_fps * param->m_time_stop;
	phase1_ratio = 1.f; 
	phase2_ratio = 1.f;
	// initialize Singleton data member for output
	{
		std::vector<btScalar> vec;

		int m = 1;
		S_dumpster::Get().test_info.reserve(m);
		for (int i = 0; i < m; i++) {
			S_dumpster::Get().test_info.push_back(vec);
			S_dumpster::Get().test_info[i].reserve(total_frame);
		}

		// to monitor these variables: pass in its address, number of elements following that address, specify the filename
		// examples:
		if (param->m_model != MODEL::TEST) {
			S_dumpster::Get().Monitor(&(m_mystacialPad->getHamiltonian()), 1, "Hamiltonian.csv", total_frame);
			S_dumpster::Get().Monitor(&(m_mystacialPad->getNasolabialis()->getFiberByIndex(0)->getExcitation()), 1, "N_excitation.csv", total_frame);
			S_dumpster::Get().Monitor(&(m_mystacialPad->getNasolabialis()->getFiberByIndex(0)->getActivation()), 1, "N_activation.csv", total_frame);

			// get ISM lengths
			for (int n = 0; n < m_mystacialPad->getNumISMs(); n++) {
				if (const auto& ism = m_mystacialPad->getISMByIndex(n)) {
					char filename[50];
					sprintf(filename, "ISM_%02d_length.csv", n);
					S_dumpster::Get().Monitor(&(ism->getLength()), 1, filename, total_frame);
				}
			}
			// get N lengths
			for (int n = 0; n < m_mystacialPad->getNasolabialis()->getNumberOfMusclePieces(); n++) {
				if (const auto& fiber = m_mystacialPad->getNasolabialis()->getFiberByIndex(n)) {
					char filename[50];
					sprintf(filename, "N_%02d_length.csv", n);
					S_dumpster::Get().Monitor(&(fiber->getLength()), 1, filename, total_frame);
				}
			}
			// get M lengths
			for (int n = 0; n < m_mystacialPad->getMaxillolabialis()->getNumberOfMusclePieces(); n++) {
				if (const auto& fiber = m_mystacialPad->getMaxillolabialis()->getFiberByIndex(n)) {
					char filename[50];
					sprintf(filename, "M_%02d_length.csv", n);
					S_dumpster::Get().Monitor(&(fiber->getLength()), 1, filename, total_frame);
				}
			}
			// get PIP lengths
			for (int n = 0; n < m_mystacialPad->getParsInternaProfunda()->getNumberOfMusclePieces(); n++) {
				if (const auto& fiber = m_mystacialPad->getParsInternaProfunda()->getFiberByIndex(n)) {
					char filename[50];
					sprintf(filename, "PIP_%02d_length.csv", n);
					S_dumpster::Get().Monitor(&(fiber->getLength()), 1, filename, total_frame);
				}
			}
			// get PM lengths
			for (int n = 0; n < m_mystacialPad->getParsMaxillaris()->getNumberOfMusclePieces(); n++) {
				if (const auto& fiber = m_mystacialPad->getParsMaxillaris()->getFiberByIndex(n)) {
					char filename[50];
					sprintf(filename, "PM_%02d_length.csv", n);
					S_dumpster::Get().Monitor(&(fiber->getLength()), 1, filename, total_frame);
				}
			}
			// get NS lengths
			if (auto& ns = m_mystacialPad->getNasolabialisSuperficialis()) {
				for (int n = 0; n < ns->getNumberOfMusclePieces(); n++) {
					if (const auto& fiber = ns->getFiberByIndex(n)) {
						char filename[50];
						sprintf(filename, "NS_%02d_length.csv", n);
						S_dumpster::Get().Monitor(&(fiber->getLength()), 1, filename, total_frame);
					}
				}
			}
			// get POO lengths
			if (auto& poo = m_mystacialPad->getParsOrbicularisOris()) {
				for (int n = 0; n < poo->getNumberOfMusclePieces(); n++) {
					if (const auto& fiber = poo->getFiberByIndex(n)) {
						char filename[50];
						sprintf(filename, "POO_%02d_length.csv", n);
						S_dumpster::Get().Monitor(&(fiber->getLength()), 1, filename, total_frame);
					}
				}
			}
			switch (param->m_model) {
			case MODEL::REDUCED:
				if (m_mystacialPad->getNumISMs() >= 3) {
					// 3 for C2 in reduced-size array
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(3)->getRestLength()), 1, "ISM_03_rest_length.csv", total_frame);
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(3)->getForce()), 1, "ISM_03_force.csv", total_frame);
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(3)->getForceHillModelComps()), 3, "ISM_03_hill_model_comps.csv", total_frame);
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(3)->getExcitation()), 1, "ISM_03_excitation.csv", total_frame);
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(3)->getActivation()), 1, "ISM_03_activation.csv", total_frame);
				}
				break;
			case MODEL::FULL:
				if (m_mystacialPad->getNumISMs() >= 12) {
					// 12 for C2 in full-size array
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(12)->getRestLength()), 1, "ISM_12_rest_length.csv", total_frame);
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(12)->getForce()), 1, "ISM_12_force.csv", total_frame);
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(12)->getForceHillModelComps()), 3, "ISM_12_hill_model_comps.csv", total_frame);
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(12)->getExcitation()), 1, "ISM_12_excitation.csv", total_frame);
					S_dumpster::Get().Monitor(&(m_mystacialPad->getISMByIndex(12)->getActivation()), 1, "ISM_12_activation.csv", total_frame);
				}
				break;
			}
		}

	}

	// alter physics properties post-initialization
	{
		m_mystacialPad->postInitPhysics();
	}

	// set up follicle position output 
	if (m_mystacialPad) {
		int nFol = m_mystacialPad->getNumFollicles();
		output_fol_pos.reserve(nFol);
		std::vector<std::vector<btScalar>> vec;
		for (int i = 0; i < nFol; i++) {
			output_fol_pos.push_back(vec);
			output_fol_pos[i].reserve(total_frame);
		}
	}

	// generate graphics
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	if (param->getArrayModel() != MODEL::TEST) {
		zeroFrameSetup();
	}
	resetCamera();

	// set exit flag to zero
	exitSim = false;

	std::cout << "\n\nStart simulation..." << std::endl;
	std::cout << "\n====================================================\n" << std::endl;
}

void Simulation::zeroFrameSetup() {
	// initialize follicle color
	double color[4] = { .6, .6, .6, 1 };
	if (m_mystacialPad) {
		int numFollicles = m_mystacialPad->getNumFollicles();
		for (int i = 0; i < numFollicles; i++) {
			if (const auto& fol = m_mystacialPad->getFollicleByIndex(i)) {
				m_guiHelper->changeRGBAColor(fol->getBody()->getUserIndex(), color);
			}
		}
	}
}

void Simulation::calculateContractionPhase() {
	btScalar t_cycle = (btScalar)(m_step - 1) / param->getFPS();
	btScalar t_phase1 = t_cycle - param->phase1_offset;
	btScalar t_phase2 = t_cycle - param->phase2_offset;
	btScalar full_cycle = 1.0;


	bool phase1_contracting = false;
	bool phase2_contracting = false;
	static int a = 0;
	if (t_phase1 >= 0) {
		btScalar t_phase1_loc = modf(t_phase1, &full_cycle);
		// reduce the phase count by 0.5 at the start (contraction) and the peak (relaxation)
		if (btEqual(t_phase1_loc, 1e-9) || btEqual(t_phase1_loc - param->phase1_peak, 1e-9)) {
			param->phase1_count -= 0.5;
			a++;
			int b = 1;
		}
		if (t_phase1_loc < param->phase1_peak) {
			phase1_contracting = true;
		}
	}
	if (t_phase2 >= 0) {
		btScalar t_phase2_loc = modf(t_phase2, &full_cycle);
		if (btEqual(t_phase2_loc, 1e-9) || btEqual(t_phase2_loc - param->phase1_peak, 1e-9)) {
			param->phase2_count -= 0.5;
		}
		if (t_phase2_loc < param->phase2_peak) {
			phase2_contracting = true;
		}
	}

	// don't update protract/retract ratios after the count is exhausted
	if (param->phase1_count >= 0) {
		phase1_ratio = 1.0f;
		if (phase1_contracting) {
			phase1_ratio = param->contract_to;
		}
	}
	if (param->phase2_count >= 0) {
		phase2_ratio = 1.0f;
		if (phase2_contracting) {
			phase2_ratio = param->contract_to;
		}
	}
}

void Simulation::updateCollisionListener() {
	std::map<const btCollisionObject*, std::vector<btManifoldPoint*>> objectsCollisions;
	int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		auto* objA = contactManifold->getBody0();
		auto* objB = contactManifold->getBody1();

		auto& vecManifoldPointsA = objectsCollisions[objA];
		auto& vecManifoldPointsB = objectsCollisions[objB];
		int numContacts = contactManifold->getNumContacts();
		if (numContacts) {
			double color[4] = { 0, 0, 0, 1 };
			//m_guiHelper->changeRGBAColor(objA->getUserIndex(), color);
			//m_guiHelper->changeRGBAColor(objB->getUserIndex(), color);
			Follicle_info* infoA = static_cast<Follicle_info*>(objA->getUserPointer());
			Follicle_info* infoB = static_cast<Follicle_info*>(objB->getUserPointer());
			std::cout << "Follicle "
				<< infoA->userIndex
				<< " is colliding with Follicle "
				<< infoB->userIndex
				<< std::endl;
		}
		for (int j = 0; j < numContacts; j++) {

			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			vecManifoldPointsA.push_back(&pt);
			vecManifoldPointsB.push_back(&pt);
		}
	}
}

void Simulation::initParameter(Parameter* parameter) {
	param = parameter;
	if (!isPathExist(param->output_path)) {	// create folder if not exist
		mkdir(param->output_path);
		std::cout << "Creating output folder..." << std::endl;
	}

	std::string createdMuscleStrings;
	for (int i = 0; i < 8; i++) {
		MUSCLE mus = static_cast<MUSCLE>(BIT(i));
		if ((param->FlagCreateMuscles & mus) == mus) {
			createdMuscleStrings += MSUCLEstrings[mus] + ", ";
		}
	}
	std::string contractedMuscleStrings;
	for (int i = 0; i < 8; i++) {
		MUSCLE mus = static_cast<MUSCLE>(BIT(i));
		if ((param->FlagContractMuscle & mus) == mus) {
			contractedMuscleStrings += MSUCLEstrings[mus] + ", ";
		}
	}

	sprintf(parameter_string,
		"FPS: %dHz\nSimulation internal step: %d\n"
		"Muscle query rate: %dHz\n"
		"Whisking freq = %.1fHz Contract to: %.2f\n"
		"Has muscles: %s\nContracting muscles:%s\n"
		"Phase 1 count = %.1f offset = %.1f peak = %.1f\n"
		"Phase 2 count = %.1f offset = %.1f peak = %.1f\n\n"
		"k_layer1 = %.1f\nk_layer2 = %.1f\nzeta_layer=%.1f\n"
		"k_anchor_translational = %.1f\nk_anchor_torsional = %.1f\n"
		"zeta_anchor_translational = %.1f\nzeta_anchor_torsional = %.1f\n\n"
		"fol_damping = %.1f\n\n"
		"muscle activation constant tau = %.2f\n"
		"muscle dactivation constant tau = %.2f\n"
		"f0 = %.2f\n"
		"ISM scaling = % d\nN scaling = %d\nM scaling = %d\n"
		"NS scaling = % d\nPMS scaling = %d\nPMI scaling = %d\n"
		"PIP scaling = % d\nPM scaling = %d\n",
		param->getFPS(), param->m_num_internal_step,
		(int)(1./param->inverse_fiber_query_rate),
		param->whisking_frequency, param->contract_to,
		createdMuscleStrings.c_str(), contractedMuscleStrings.c_str(),
		param->phase1_count, param->phase1_offset, param->phase1_peak,
		param->phase2_count, param->phase2_offset, param->phase2_peak,
		param->k_layer1, param->k_layer2, param->zeta_layer,
		param->k_anchor_translational, param->k_anchor_torsional,
		param->zeta_anchor_translational, param->zeta_anchor_torsional,
		param->fol_damping,
		param->muslce_activation_tau_a, param->muslce_activation_tau_d,
		param->f0,
		(int)(param->f0_ISM / param->f0), (int)( param->f0_nasolabialis / param->f0), (int)(param->f0_maxillolabialis / param->f0),
		(int)(param->f0_NS / param->f0), (int)(param->f0_PMS / param->f0), (int)(param->f0_PMI / param->f0),
		(int)(param->f0_PIP / param->f0), (int)( param->f0_PM / param->f0)
	);
}

const std::string& Simulation::getDrawText() const {
	return draw_text;
}

void Simulation::internalWriteOutput() {
	if (m_mystacialPad && param->getArrayModel() != MODEL::TEST) {
		// clear the directory
		for (const auto& entry : std::filesystem::directory_iterator(param->output_path)) {
			try {
				std::filesystem::remove_all(entry.path());
			}
			catch (...) {
				// mp4 will throw an exception because it is open
				// do nothing for mp4 file
			}
		}

		std::cout << "Generating output csv file...\n";

		int nFol = m_mystacialPad->getNumFollicles();

		char filename[50];
		for (int i = 0; i < nFol; i++) {
			sprintf(filename, "fol_%02d.csv\0", i);
			//std::cout << filename << std::endl;
			write_csv_float(param->output_path, filename, output_fol_pos[i]);
		}
		write_txt(param->output_path, "parameter.txt", parameter_string);

		S_dumpster::Get().Output(param->output_path);

		std::cout << "Files saved.\n";
	}
}

void Simulation::applyAdditionalDamping() {
	m_mystacialPad->applyAdditionalDamping();
}

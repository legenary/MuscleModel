#include "my_pch.h"
#include "Simulation.h"

#include "Parameter.h"
#include "Instrumentor.h"
#include "Utility.h"
#include "MystacialPad.h"
#include "IntrinsicMuscle.h"
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

		// contraction/retraction
		int numStepToChangeState = param->getFPS() / param->contract_frequency / 2.0f;
		flagMuscleContractionStateChange = (m_step - 1) % numStepToChangeState == 0;

		static btScalar contractTo = 1 - param->contract_range;
		if (flagMuscleContractionStateChange && param->contract_count > 0) {

			// contarct synchronously with ISM (these muscles are protractors)
			m_mystacialPad->contractMuscle(MUSCLE::ISM, contractTo);
			m_mystacialPad->contractMuscle(MUSCLE::NS, contractTo);
			m_mystacialPad->contractMuscle(MUSCLE::PMS, contractTo);
			m_mystacialPad->contractMuscle(MUSCLE::PMI, contractTo);
			m_mystacialPad->contractMuscle(MUSCLE::N, contractTo);
			m_mystacialPad->contractMuscle(MUSCLE::M, contractTo);
			m_mystacialPad->contractMuscle(MUSCLE::PIP, contractTo);
			m_mystacialPad->contractMuscle(MUSCLE::PM, contractTo);
			contractTo = 2 - param->contract_range - contractTo;
			// contract asynchronously with ISM (these muscles are retractors)

			// reduced contract count by half cycle
			param->contract_count -= 0.5;
		}

		// update constraint physics options
		m_mystacialPad->update(deltaTime);

		// last step: step simulation
		{
			PROFILE_SCOPE("m_dynamicsWorld->StepSimulation()");
			m_dynamicsWorld->stepSimulation(deltaTime,	// rendering time step
				param->m_num_internal_step * 100,		// max sub step
				param->m_internal_time_step);			// fixed simulation sub time step
		}

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

	box1 = createDynamicBody(1, createTransform(btVector3(5, 0, 0)), boxShape);
	m_dynamicsWorld->addRigidBody(box1, COL_FOLLICLE, follicleCollideWith);
	box1->setActivationState(DISABLE_DEACTIVATION);

	box2 = createDynamicBody(1, createTransform(btVector3(-5, 0, 0)), boxShape);
	m_dynamicsWorld->addRigidBody(box2, COL_FOLLICLE, follicleCollideWith);
	box2->setActivationState(DISABLE_DEACTIVATION);

	box1->setAngularVelocity(btVector3(0, 0, 2));

	//// add constraints
	//btScalar k = 1;
	//btScalar f0 = 10;
	//btScalar zeta = 2;
	//t1 = new Tissue(this, box1, 
	//	createTransform(btVector3(0, 0, 0)), k, zeta); // critical damping ratio is 1 for 1 mass
	//m_dynamicsWorld->addConstraint(t1->getConstraint(), true);

	//t2 = new Tissue(this, box2, 
	//	createTransform(btVector3(-5, 0, 0)), k, zeta); // critical damping ratio is 1 for 1 mass
	//m_dynamicsWorld->addConstraint(t2->getConstraint(), true);

	//f = new Fiber(this, box1, box2, createTransform(), createTransform(), f0);
	//m_dynamicsWorld->addConstraint(f->getConstraint(), true);
	//
	////box1->setCenterOfMassTransform(createTransform(btVector3(-5, 0, 0)));

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
		S_dumpster::Get().Monitor(&(m_mystacialPad->getHamiltonian()), 1, "Hamiltonian.csv", total_frame);
		for (int n = 0; n < m_mystacialPad->getNumISMs(); n++) {
			if (const auto& ism = m_mystacialPad->getISMByIndex(n)) {
				char filename[50];
				sprintf(filename, "ISM_%02d_length.csv", n);
				S_dumpster::Get().Monitor(&(ism->getLength()), 1, filename, total_frame);
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
			m_guiHelper->changeRGBAColor(objA->getUserIndex(), color);
			m_guiHelper->changeRGBAColor(objB->getUserIndex(), color);
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

	sprintf(parameter_string,
		"FPS: %dHz\nSimulation internal step: %d\n"
		"Muscle query rate: %.1fHz\n"
		"Contraction ratio: %.2f\n"
		"Tissue damping ratio zeta = %.3f\n"
		"Anchor damping ratio zeta (translational) = %.3f\n"
		"Anchor damping ratio zeta (torsional) = %.3f\n"
		"k_layer1=%.6fN/m\nk_layer2=%.6fN/m\n\n"
		"fo unit: %.6fN\nf0_intrinsic = %.6fN\n"
		"f0_nasolabialis(N)=%.6fN\nf0_maxillolabialis(M)=%.6fN\n"
		"f0_nasolabialis_superficialis(NS)=%.6fN\n"
		"f0_pars_media_superior=%.6fN\nf0_pars_media_inferior=%.6fN\n"
		"f0_pars_interna_profunda=%.6fN\nf0_pars_maxillaris=%.6fN\n",
		param->getFPS(), param->m_num_internal_step,
		1 / param->inverse_fiber_query_rate,
		param->contract_range,
		param->zeta_layer,
		param->zeta_anchor_translational, param->zeta_anchor_torsional,
		param->k_layer1 * 0.001, param->k_layer2 * 0.001,
		param->f0_ISM * 0.000001 / 20,
		param->f0_ISM * 0.000001,
		param->f0_nasolabialis * 0.000001, param->f0_maxillolabialis * 0.000001,
		param->f0_NS * 0.000001, param->f0_PMS * 0.000001,
		param->f0_PMI * 0.000001, param->f0_PIP * 0.000001, param->f0_PM * 0.000001
	);
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

#include "my_pch.h"
#include "Simulation.h"

#include "Parameter.h"
#include "Utility.h"
#include "MystacialPad.h"

#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "CommonInterfaces/CommonParameterInterface.h"

#define BIT(x) (1<<(x))

Simulation::~Simulation() {
	delete m_mystacialPad;
}

void Simulation::stepSimulation(float deltaTime) {
	auto start = std::chrono::high_resolution_clock::now();
	m_time += param->m_time_step; 						// increase time
	m_step += 1;										// increase step

	if (param->m_time_stop == 0 || m_time < param->m_time_stop) {
		// update physics
		if (param->contractISM) {
			std::vector<int> those = { 12, 13 }; // specify which muscle to contract: see modeling_log.pptx
			m_mystacialPad->contractIntrinsicSlingMuscle(m_step, param, those);
		}
		if (param->contractNasolabialis) {
			m_mystacialPad->contractNasolabialis(m_step, param);
		}
		if (param->contractMaxillolabialis) {
			m_mystacialPad->contractMaxillolabialis(m_step, param);
		}
		m_mystacialPad->update();
		m_mystacialPad->debugDraw(m_dynamicsWorld, param->DEBUG);

		// last step: step simulation
		m_dynamicsWorld->stepSimulation(deltaTime, param->m_num_internal_step,
			param->m_time_step / param->m_num_internal_step);

		if (param->DEBUG) {
			m_dynamicsWorld->debugDrawWorld();
		}

		// set exit flag to zero
		exitSim = false;
	}
	else {
		write_csv_float("../output", "test_output.csv", output);

		// timeout -> set exit flag
		exitSim = true;
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	m_time_elapsed += duration.count() / 1000.f;
	auto factor = m_time_elapsed / m_time;
	auto time_remaining = (int)((param->m_time_stop - m_time) * (factor));


}

void Simulation::initParameter(Parameter *parameter) {
	param = parameter;
}

void Simulation::initPhysics() {
	std::cout << "Physics engine initiating...." << std::endl;
	// set visual axis
	m_guiHelper->setUpAxis(2);

	// create empty dynamics world[0]
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	// broadphase algorithm
	m_broadphase = new btDbvtBroadphase();

	// select solver
	std::cout << "Using btSequentialImpulseConstraintSolver..." << std::endl;
	m_solver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

	// set number of iterations
	m_dynamicsWorld->getSolverInfo().m_numIterations = 20;
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


	// Initializing physics world
	////////////////////////////////////////////////////////////////////////////////
	read_csv_float(param->dir_follicle_pos_orient_len_vol, param->FOLLICLE_POS_ORIENT_LEN_VOL);
	m_mystacialPad = new MystacialPad(m_dynamicsWorld, &m_collisionShapes, param);

	// layers
	read_csv_int(param->dir_spring_hex_mesh_idx, param->SPRING_HEX_MESH_IDX);
	m_mystacialPad->createLayer1(m_dynamicsWorld, param);
	m_mystacialPad->createLayer2(m_dynamicsWorld, param);
	m_mystacialPad->createAnchor(m_dynamicsWorld, param);

	//// intrinsic sling muscles
	//read_csv_int(param->dir_intrinsic_sling_muscle_idx, param->INTRINSIC_SLING_MUSCLE_IDX);
	//read_csv_float(param->dir_intrinsic_sling_muscle_contraction_trajectory, param->INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY);
	//m_mystacialPad->createIntrinsicSlingMuscle(m_dynamicsWorld, param);
	//
	// extrinsic: nasolabialis muscle
	read_csv_float(param->dir_nasolabialis_node_pos, param->NASOLABIALIS_NODE_POS);
	read_csv_int(param->dir_nasolabialis_construction_idx, param->NASOLABIALIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_nasolabialis_insertion_idx, param->NASOLABIALIS_INSERTION_IDX);
	read_csv_float(param->dir_nasolabialis_contraction_trajectory, param->NASOLABIALIS_CONTRACTION_TRAJECTORY);
	//m_mystacialPad->createNasolabialis(m_dynamicsWorld, &m_collisionShapes, param);

	// extrinsic: maxillolabialis muscle
	read_csv_float(param->dir_maxillolabialis_node_pos, param->MAXILLOLABIALIS_NODE_POS);
	read_csv_int(param->dir_maxillolabialis_construction_idx, param->MAXILLOLABIALIS_CONSTRUCTION_IDX);
	read_csv_int(param->dir_maxillolabialis_insertion_idx, param->MAXILLOLABIALIS_INSERTION_IDX);
	read_csv_float(param->dir_maxillolabialis_contraction_trajectory, param->MAXILLOLABIALIS_CONTRACTION_TRAJECTORY);
	//m_mystacialPad->createMaxillolabialis(m_dynamicsWorld, &m_collisionShapes, param);

	//// extrinsic: nasolabialis superficialis
	//read_csv_float(param->dir_nasolabialis_superficialis_node_pos, param->NASOLABIALIS_SUPERFICIALIS_NODE_POS);
	//read_csv_int(param->dir_nasolabialis_superficialis_construction_idx, param->NASOLABIALIS_SUPERFICIALIS_CONSTRUCTION_IDX);
	//read_csv_int(param->dir_nasolabialis_superficialis_insertion_idx, param->NASOLABIALIS_SUPERFICIALIS_INSERTION_IDX);
	//m_mystacialPad->createNasolabialisSuperficialis(m_dynamicsWorld, &m_collisionShapes, param);

	//// extrinsic: pars media superior of M. Nasolabialis profundus
	//read_csv_float(param->dir_pars_media_superior_node_pos, param->PARS_MEDIA_SUPERIOR_NODE_POS);
	//read_csv_int(param->dir_pars_media_superior_construction_idx, param->PARS_MEDIA_SUPERIOR_CONSTRUCTION_IDX);
	//read_csv_int(param->dir_pars_media_superior_insertion_idx, param->PARS_MEDIA_SUPERIOR_INSERTION_IDX);
	//read_csv_float(param->dir_pars_media_superior_insertion_height, param->PARS_MEDIA_SUPERIOR_INSERTION_HEIGHT);
	//m_mystacialPad->createParsMediaSuperior(m_dynamicsWorld, &m_collisionShapes, param);

	//// extrinsic: pars media inferior of M. Nasolabialis profundus
	//read_csv_float(param->dir_pars_media_inferior_node_pos, param->PARS_MEDIA_INFERIOR_NODE_POS);
	//read_csv_int(param->dir_pars_media_inferior_construction_idx, param->PARS_MEDIA_INFERIOR_CONSTRUCTION_IDX);
	//read_csv_int(param->dir_pars_media_inferior_insertion_idx, param->PARS_MEDIA_INFERIOR_INSERTION_IDX);
	//read_csv_float(param->dir_pars_media_inferior_insertion_height, param->PARS_MEDIA_INFERIOR_INSERTION_HEIGHT);
	//m_mystacialPad->createParsMediaInferior(m_dynamicsWorld, &m_collisionShapes, param);

	//// extrinsic: pars interna profunda of M. Nasolabialis profundus
	//read_csv_float(param->dir_pars_interna_profunda_node_pos, param->PARS_INTERNA_PROFUNDA_NODE_POS);
	//read_csv_int(param->dir_pars_interna_profunda_construction_idx, param->PARS_INTERNA_PROFUNDA_CONSTRUCTION_IDX);
	//read_csv_int(param->dir_pars_interna_profunda_insertion_idx, param->PARS_INTERNA_PROFUNDA_INSERTION_IDX);
	//read_csv_float(param->dir_pars_interna_profunda_insertion_height, param->PARS_INTERNA_PROFUNDA_INSERTION_HEIGHT);
	//m_mystacialPad->createParsInternaProfunda(m_dynamicsWorld, &m_collisionShapes, param);

	//// extrinsic: pars maxillaris of M. Nasolabialis profundus
	//read_csv_float(param->dir_pars_maxillaris_node_pos, param->PARS_MAXILLARIS_NODE_POS);
	//read_csv_int(param->dir_pars_maxillaris_construction_idx, param->PARS_MAXILLARIS_CONSTRUCTION_IDX);
	//read_csv_int(param->dir_pars_maxillaris_insertion_idx, param->PARS_MAXILLARIS_INSERTION_IDX);
	//read_csv_float(param->dir_pars_maxillaris_insertion_height, param->PARS_MAXILLARIS_INSERTION_HEIGHT);
	//m_mystacialPad->createParsMaxillaris(m_dynamicsWorld, &m_collisionShapes, param);


	////////////////////////////////////////////////////////////////////////////////

	// generate graphics
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	resetCamera();

	// initialize time/step tracker

	std::cout << "\n\nStart simulation..." << std::endl;
	std::cout << "\n====================================================\n" << std::endl;
}


void Simulation::resetCamera() {
	m_guiHelper->resetCamera(param->camDist, param->camYaw, param->camPitch, 
							 param->camPos[0], param->camPos[1], param->camPos[2]);
}


void Simulation::initPhysics_test() {
	std::cout << "Physics engine initiating...." << std::endl;
	// set visual axis
	m_guiHelper->setUpAxis(2);

	// create empty dynamics world[0]
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	// broadphase algorithm
	m_broadphase = new btDbvtBroadphase();

	// select solver
	std::cout << "Using btSequentialImpulseConstraintSolver..." << std::endl;
	m_solver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

	// set number of iterations
	m_dynamicsWorld->getSolverInfo().m_numIterations = 20;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD |
		SOLVER_USE_WARMSTARTING |
		SOLVER_RANDMIZE_ORDER |
		0;
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
	m_dynamicsWorld->getSolverInfo().m_erp = 0.8f;

	// set gravity
	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));


	btCollisionShape* boxShape = new btBoxShape(btVector3(1, 1, 1));
	m_collisionShapes.push_back(boxShape);
	box1 = createDynamicBody(1, createTransform(btVector3(0, 0, 0)), boxShape);
	box2 = createDynamicBody(1, createTransform(btVector3(-5, 0, 0)), boxShape);
	m_dynamicsWorld->addRigidBody(box1, COL_FOLLICLE, follicleCollideWith);
	m_dynamicsWorld->addRigidBody(box2, COL_FOLLICLE, follicleCollideWith);
	box1->setActivationState(DISABLE_DEACTIVATION);
	box2->setActivationState(DISABLE_DEACTIVATION);



	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	resetCamera();
}

void Simulation::stepSimulation_test(float deltaTime) {
	auto start = std::chrono::high_resolution_clock::now();
	m_time += param->m_time_step; 						// increase time
	m_step += 1;										// increase step

	if (param->m_time_stop == 0 || m_time <= param->m_time_stop) {
		// update everything here:
		btVector3 force(0.1, 0, 0);
		box1->applyCentralForce(force);
		//box1->applyCentralImpulse(force * param->m_time_step);
		btVector3 box1pos = box1->getCenterOfMassPosition();
		std::vector<float> vect{ box1pos[0], box1pos[1], box1pos[2] };
		output.push_back(vect);


		// ask world to step simulation
		m_dynamicsWorld->stepSimulation(deltaTime, param->m_num_internal_step,
			param->m_time_step / param->m_num_internal_step);

		if (param->DEBUG) {
			m_dynamicsWorld->debugDrawWorld();
		}

		// set exit flag to zero
		exitSim = false;
	}
	else {
		write_csv_float("../output", "test_output.csv", output);
		// timeout -> set exit flag
		exitSim = true;
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	m_time_elapsed += duration.count() / 1000.f;
	auto factor = m_time_elapsed / m_time;
	auto time_remaining = (int)((param->m_time_stop - m_time) * (factor));


}
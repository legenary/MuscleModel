#include "Simulation.h"

#define BIT(x) (1<<(x))

void Simulation::stepSimulation() {
	auto start = std::chrono::high_resolution_clock::now();
	m_time += param->m_time_step; 						// increase time
	m_step += 1;										// increase step

	if (m_time > (param->m_time_stop / 2.)) {
		//std::cout << m_mystacialPad->getNumFollicles();
		//std::cout << param->SPRING_HEX_MESH_INDEX[3][1] << std::endl;
		//m_mystacialPad->getFollicleByIndex(0)->getBody()->setLinearVelocity(btVector3(1, 0, 0));
	}

	if (param->m_time_stop == 0 || m_time < param->m_time_stop) {
		// update physics
		if (param->contractISM) {
			m_mystacialPad->contract(m_step, param);
		}
		m_mystacialPad->update();
		m_mystacialPad->debugDraw(m_dynamicsWorld, param->DEBUG);

		// last step: step simulation
		m_dynamicsWorld->stepSimulation(param->m_time_step, param->m_num_internal_step,
			param->m_time_step / param->m_num_internal_step);

		if (param->DEBUG) {
			m_dynamicsWorld->debugDrawWorld();
		}

		// set exit flag to zero
		exitSim = 0;
	}
	else {
		// timeout -> set exit flag
		exitSim = 1;
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

	// set debug drawer
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

	//m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	//m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_NoDebug);

	// Initializing physics world
	////////////////////////////////////////////////////////////////////////////////
	read_csv_float(param->dir_follicle_loc_orient, param->FOLLICLE_LOC_ORIENT);
	m_mystacialPad = new MystacialPad(m_dynamicsWorld, &m_collisionShapes, param);

	read_csv_int(param->dir_spring_hex_mesh_index, param->SPRING_HEX_MESH_INDEX);
	m_mystacialPad->createLayer1(m_dynamicsWorld, param);
	m_mystacialPad->createLayer2(m_dynamicsWorld, param);
	m_mystacialPad->createAnchor(m_dynamicsWorld, param);
	read_csv_int(param->dir_intrinsic_sling_muscle_index, param->INTRINSIC_SLING_MUSCLE_INDEX);
	m_mystacialPad->createIntrinsicSlingMuscle(m_dynamicsWorld, param);
	read_csv_float(param->dir_intrinsic_sling_muscle_contraction_trajectory, param->INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY);


	//m_mystacialPad->getFollicleByIndex(0)->getBody()->setLinearVelocity(btVector3(0, 0, 5));

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

//Simulation* SimulationCreateFunc(CommonExampleOptions& options)
//{
//	return new Simulation(options.m_guiHelper);
//}

void Simulation::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

//B3_STANDALONE_EXAMPLE(SimulationCreateFunc)
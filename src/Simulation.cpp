#include "Simulation.h"

#define BIT(x) (1<<(x))

void Simulation::stepSimulation() {
	auto start = std::chrono::high_resolution_clock::now();
	m_time += param->m_time_step; 								// increase time
	m_step += 1;													// increase step

	if (m_time > (param->m_time_stop/20.)) {
		// std::cout << param->SPRING_HEX_MESH_INDEX[3][1] << std::endl;
		//m_pad->getFollicleByIndex(0)->getBody()->setLinearVelocity(btVector3(1, 0, 0));
	}

	if (m_time < param->m_time_stop) {
		
		// step simulation
		m_dynamicsWorld->stepSimulation(param->m_time_step, param->m_num_internal_step, 
										param->m_time_step / param->m_num_internal_step);

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

	//m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	//m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_NoDebug);

	// Initializing physics world
	////////////////////////////////////////////////////////////////////////////////
	read_csv_float(param->dir_follicle_loc_orient, param->FOLLICLE_LOC_ORIENT);
	MystacialPad* m_pad = new MystacialPad(m_dynamicsWorld, &m_collisionShapes, param);

	read_csv_int(param->dir_spring_hex_mesh_index, param->SPRING_HEX_MESH_INDEX);
	m_pad->createLayer1(m_dynamicsWorld, &m_collisionShapes, param);
	m_pad->createLayer2(m_dynamicsWorld, &m_collisionShapes, param);
	//m_pad->test(m_dynamicsWorld, &m_collisionShapes, param);

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

Simulation* SimulationCreateFunc(CommonExampleOptions& options)
{
	return new Simulation(options.m_guiHelper);
}

void Simulation::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

//B3_STANDALONE_EXAMPLE(SimulationCreateFunc)
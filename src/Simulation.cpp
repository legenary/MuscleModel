#include "Simulation.h"

void Simulation::stepSimulation() {
	auto start = std::chrono::high_resolution_clock::now();
	m_time += m_time_step; 								// increase time
	m_step += 1;													// increase step

	if (m_time < 5./60) {
		std::cout << "step." << std::endl;
	}

	if (m_time < 100.) {

		
		// step simulation
		m_dynamicsWorld->stepSimulation(m_time_step, 10, m_time_step / 10);

		// set exit flag to zero
		exitSim = 0;
	}
	else {
		// timeout -> set exit flg
		exitSim = 1;
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	m_time_elapsed += duration.count() / 1000.f;
	auto factor = m_time_elapsed / m_time;
	auto time_remaining = (int)((10. - m_time) * (factor));
	
}

void Simulation::initPhysics()
{
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
	//m_dynamicsWorld->setGravity(btVector3(0, 0, 0));

	// generate graphics
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	// set camera position to rat head
	camPos[0] = 0;
	camPos[1] = 0;
	camPos[2] = 0;
	camDist = 50;
	camPitch = -89;
	camYaw = 0;
	resetCamera();

	// initialize time/step tracker

	std::cout << "\n\nStart simulation..." << std::endl;
	std::cout << "\n====================================================\n" << std::endl;
}

void Simulation::resetCamera() {
	m_guiHelper->resetCamera(camDist, camYaw, camPitch, camPos[0], camPos[1], camPos[2]);
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
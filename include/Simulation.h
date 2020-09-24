#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <chrono> 
#include <iomanip>      // std::setprecision

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btQuaternion.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "CommonInterfaces/CommonParameterInterface.h"

class Simulation* SimulationCreateFunc(struct CommonExampleOptions& options);
//class Simulation* StandaloneExampleCreateFunc(struct CommonExampleOptions& options);

class Simulation : public CommonRigidBodyBase {

private:
	btScalar m_time_step = 1. / 60.;

	btScalar m_time_elapsed;
	btScalar m_time;
	int m_step;


public:
	Simulation(struct GUIHelperInterface* helper) :CommonRigidBodyBase(helper) {}
	virtual ~Simulation() {}
	virtual void initPhysics();
	virtual void renderScene();
	virtual void stepSimulation();

	btScalar camPos[3];
	btScalar camDist;
	btScalar camPitch;
	btScalar camYaw;
	void resetCamera();

	bool exitSim;
};














#endif
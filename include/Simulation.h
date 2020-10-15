#ifndef SIMULATION_H
#define SIMULATION_H

#include "Parameter.h"
#include "MystacialPad.h"

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
	Parameter *param;
	btScalar m_time_elapsed = 0.;
	btScalar m_time = 0.;
	int m_step = 0;


public:
	Simulation(struct GUIHelperInterface* helper) :CommonRigidBodyBase(helper) {}
	virtual ~Simulation() {}
	void initParameter(Parameter* parameter);
	virtual void initPhysics();
	virtual void renderScene();
	virtual void stepSimulation();
	
	void resetCamera();

	bool exitSim;

	btRigidBody* body1;

};














#endif
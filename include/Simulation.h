#ifndef SIMULATION_H
#define SIMULATION_H

class Parameter;
class MystacialPad;

#include "CommonInterfaces/CommonRigidBodyBase.h"


class Simulation* SimulationCreateFunc(struct CommonExampleOptions& options);
//class Simulation* StandaloneExampleCreateFunc(struct CommonExampleOptions& options);

class Simulation : public CommonRigidBodyBase {

private:
	Parameter *param; 
	MystacialPad* m_mystacialPad;
	btScalar m_time_elapsed = 0.;
	btScalar m_time = 0.;
	int m_step = 0;

	std::vector<std::vector<float>> output;

	/*btRigidBody* box1;
	btRigidBody* box2;
	Spring* spring;*/
	

public:
	Simulation(struct GUIHelperInterface* helper) :CommonRigidBodyBase(helper) {}
	virtual ~Simulation();
	void initParameter(Parameter* parameter);
	virtual void initPhysics();
	virtual void renderScene();
	virtual void stepSimulation();
	
	void resetCamera();

	bool exitSim;

	

};














#endif
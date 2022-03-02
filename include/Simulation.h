#ifndef SIMULATION_H
#define SIMULATION_H

class Parameter;
class MystacialPad;

#include "CommonInterfaces/CommonRigidBodyBase.h"


//class Simulation* SimulationCreateFunc(struct CommonExampleOptions& options);
//class Simulation* StandaloneExampleCreateFunc(struct CommonExampleOptions& options);

class Simulation : public CommonRigidBodyBase {

private:
	btScalar m_time_elapsed;
	btScalar m_time;
	int m_step;

	Parameter *param; 
	MystacialPad* m_mystacialPad;

	std::vector<std::vector<float>> output;

public:
	Simulation(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper), m_time_elapsed(0.), m_time(0.), m_step(0) {}
	Simulation(const Simulation&) = delete;
	Simulation& operator=(Simulation const&) = delete;
	virtual ~Simulation();
	void initParameter(Parameter* parameter);
	virtual void initPhysics() override;
	virtual void stepSimulation(float deltaTime) override;
	
	void resetCamera();
	bool exitSim = false;

};














#endif
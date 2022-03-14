#ifndef SIMULATION_H
#define SIMULATION_H

class Parameter;
class MystacialPad;
class Fiber;

#include "CommonInterfaces/CommonRigidBodyBase.h"


//class Simulation* SimulationCreateFunc(struct CommonExampleOptions& options);
//class Simulation* StandaloneExampleCreateFunc(struct CommonExampleOptions& options);

class Simulation : public CommonRigidBodyBase {

private:
	btScalar m_time_elapsed;
	btScalar m_time;
	int m_step;

	btRigidBody* box1;
	btRigidBody* box2;
	Fiber* fiber;

	Parameter *param; 
	MystacialPad* m_mystacialPad;

	std::vector<std::vector<float>> output;

public:
	Simulation(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper), m_time_elapsed(0.), m_time(0.), m_step(0) {}
	Simulation(const Simulation&) = delete;
	Simulation& operator=(Simulation const&) = delete;
	virtual ~Simulation();
	// override new and delete operator to make sure object is aligned 16 on the heap
	void* operator new(size_t i) { return _mm_malloc(i, 16); }
	void operator delete(void* p) { _mm_free(p); }


	void initParameter(Parameter* parameter);
	virtual void initPhysics() override;
	virtual void stepSimulation(float deltaTime) override;

	void initPhysics_test();
	void stepSimulation_test(float deltaTime);
	
	void resetCamera();
	bool exitSim = false;

	inline Parameter* getParameter() const { return param; }

};













#endif
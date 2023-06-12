#ifndef SIMULATION_H
#define SIMULATION_H

class Parameter;
class MystacialPad;
class Fiber;
class Tissue;

#include "CommonInterfaces/CommonRigidBodyBase.h"


//class Simulation* SimulationCreateFunc(struct CommonExampleOptions& options);
//class Simulation* StandaloneExampleCreateFunc(struct CommonExampleOptions& options);

class Simulation : public CommonRigidBodyBase {

private:
	btScalar m_time_elapsed;
	btScalar m_time;
	int m_step;

	Parameter* param; 
	MystacialPad* m_mystacialPad;
	std::vector<std::vector<std::vector<btScalar>>> output_fol_pos;
	char parameter_string[512];

	void zeroFrameSetup();

	void preInitPhysics();
	void postInitPhysics();

	void updateCollisionListener();

public:
	Simulation(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper), m_time_elapsed(0.), m_time(0.), m_step(0) {
		m_mystacialPad = nullptr;
	}
	Simulation(const Simulation&) = delete;
	Simulation& operator=(Simulation const&) = delete;
	virtual ~Simulation();

	// override new and delete operator to make sure object is aligned 16 on the heap
	void* operator new(size_t i) { return _mm_malloc(i, 16); }
	void operator delete(void* p) { _mm_free(p); }

	void initParameter(Parameter* parameter);

	virtual void initPhysics() final;
	void initPhysics_reduced();
	void initPhysics_test();

	virtual void stepSimulation(float deltaTime) final;
	void stepSimulation_test(float deltaTime);
	
	void internalWriteOutput();
	void resetCamera();
	bool exitSim = false;

	inline Parameter* getParameter() const { return param; }

	bool muscleContractionStateChanged = false;

// test
private:
	btRigidBody* box1 = nullptr;
	btRigidBody* box2 = nullptr;
	Tissue* t1 = nullptr;
	Tissue* t2 = nullptr;
	Fiber* f = nullptr;

};













#endif
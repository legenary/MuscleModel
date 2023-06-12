#ifndef MYSTACIAL_PAD_H
#define MYSTACIAL_PAD_H

#include "Simulation.h"

class Follicle;
class Tissue;
class Layer;
class IntrinsicSlingMuscle;
class ExtrinsicMuscle;
class Parameter;

enum class Muscle { INTRINSIC, N, M, NS, PMS, PMI, PIP, PM };

class MystacialPad {
private:
	Simulation* m_sim;
	Parameter* m_parameter;

	btAlignedObjectArray<Follicle*> m_follicleArray;
	Layer* m_layer1;
	Layer* m_layer2;
	btAlignedObjectArray<IntrinsicSlingMuscle*> m_ISMArray;
	btAlignedObjectArray<btRigidBody*> m_ISM_nodes;
	int nFollicle;
	int nISM;
	ExtrinsicMuscle* m_nasolabialis;
	ExtrinsicMuscle* m_maxillolabialis;
	ExtrinsicMuscle* m_NS;
	ExtrinsicMuscle* m_PMS;
	ExtrinsicMuscle* m_PMI;
	ExtrinsicMuscle* m_PIP;
	ExtrinsicMuscle* m_PM;
	std::vector<std::vector<float>> heightPlaceHolder;

	btScalar m_Hamiltonian;

public:
	MystacialPad(Simulation* sim, Parameter* param);
	// disable copy constructor (override if needed in the future)
	MystacialPad(const MystacialPad&) = delete;
	MystacialPad& operator=(MystacialPad const&) = delete;
	virtual ~MystacialPad();

	void createLayer1();
	void createLayer2();
	void createIntrinsicSlingMuscle();
	void createNasolabialis();
	void createMaxillolabialis();
	void createNasolabialisSuperficialis();
	void createParsMediaSuperior();
	void createParsMediaInferior();
	void createParsInternaProfunda();
	void createParsMaxillaris();

	void contractMuscle(Muscle mus, btScalar ratio);

	void update(btScalar dt);
	void readOutput(std::vector<std::vector<std::vector<btScalar>>>& output);
	
	int getNumFollicles() const;
	Follicle* getFollicleByIndex(int idx);

	//int getLayer1Tissue() const;
	//int getLayer2Tissue() const;

	void debugDraw();
	inline btDynamicsWorld* getWorld() {
		return m_sim->getDynamicsWorld();
	}
	inline btAlignedObjectArray<btCollisionShape*>* getCollisionShapes() {
		return &(m_sim->m_collisionShapes);
	}
	inline Follicle* getFollicleByIndex(int i) const {
		return m_follicleArray[i];
	}
	
};





#endif
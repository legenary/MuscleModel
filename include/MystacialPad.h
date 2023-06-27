#ifndef MYSTACIAL_PAD_H
#define MYSTACIAL_PAD_H

#include "Simulation.h"

class Follicle;
class Tissue;
class Layer;
class IntrinsicSlingMuscle;
class ExtrinsicMuscle;
class Parameter;
enum MUSCLE;

class MystacialPad {
private:
	Simulation* m_sim;
	Parameter* m_parameter;

	std::vector<std::unique_ptr<Follicle>> m_follicleArray;
	int nFollicle;
	std::unique_ptr<Layer> m_layer1;
	std::unique_ptr<Layer> m_layer2;
	std::vector<std::unique_ptr<IntrinsicSlingMuscle>> m_ISMArray;
	int nISM;
	std::unique_ptr<ExtrinsicMuscle> m_nasolabialis;
	std::unique_ptr<ExtrinsicMuscle> m_maxillolabialis;
	std::unique_ptr<ExtrinsicMuscle> m_NS;
	std::unique_ptr<ExtrinsicMuscle> m_PMS;
	std::unique_ptr<ExtrinsicMuscle> m_PMI;
	std::unique_ptr<ExtrinsicMuscle> m_PIP;
	std::unique_ptr<ExtrinsicMuscle> m_PM;
	std::vector<std::vector<float>> heightPlaceHolder;

	btScalar m_Hamiltonian;

	bool checkCreateMuscleFlag(MUSCLE mus) const;

public:
	MystacialPad(Simulation* sim, Parameter* param);
	// disable copy constructor (override if needed in the future)
	MystacialPad(const MystacialPad&) = delete;
	MystacialPad& operator=(MystacialPad const&) = delete;

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

	void contractMuscle(MUSCLE mus, btScalar ratio);

	void update(btScalar dt);
	void bufferFolPos(std::vector<std::vector<std::vector<btScalar>>>& output);
	
	int getNumFollicles() const;
	std::unique_ptr<Follicle>& getFollicleByIndex(int idx);

	btScalar getHamiltonian() const { return m_Hamiltonian; }

	//int getLayer1Tissue() const;
	//int getLayer2Tissue() const;

	void debugDraw();
	inline btDynamicsWorld* getWorld() {
		return m_sim->getDynamicsWorld();
	}
	inline btAlignedObjectArray<btCollisionShape*>* getCollisionShapes() {
		return &(m_sim->m_collisionShapes);
	}
	
	

};





#endif
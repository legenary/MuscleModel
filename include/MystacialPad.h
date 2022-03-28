#ifndef MYSTACIAL_PAD_H
#define MYSTACIAL_PAD_H

#include "Simulation.h"

class Follicle;
class Tissue;
class IntrinsicSlingMuscle;
class ExtrinsicMuscle;
class Parameter;

class MystacialPad {
private:
	Simulation* m_sim;
	Parameter* m_parameter;

	btAlignedObjectArray<Follicle*> m_follicleArray;
	btAlignedObjectArray<Tissue*> m_layer1;
	btAlignedObjectArray<Tissue*> m_layer2;
	btAlignedObjectArray<Tissue*> m_layer3;
	btAlignedObjectArray<Tissue*> m_anchor;
	btAlignedObjectArray<IntrinsicSlingMuscle*> m_ISMArray;
	int nFollicle;
	int nTissueLayer1;
	int nTissueLayer2;
	int nTissueAnchor;
	int nISM;
	ExtrinsicMuscle* m_nasolabialis;
	ExtrinsicMuscle* m_maxillolabialis;
	ExtrinsicMuscle* m_NS;
	ExtrinsicMuscle* m_PMS;
	ExtrinsicMuscle* m_PMI;
	ExtrinsicMuscle* m_PIP;
	ExtrinsicMuscle* m_PM;
	std::vector<std::vector<float>> heightPlaceHolder;


public:
	MystacialPad(Simulation* sim, Parameter* param);
	// disable copy constructor (override if needed in the future)
	MystacialPad(const MystacialPad&) = delete;
	MystacialPad& operator=(MystacialPad const&) = delete;
	virtual ~MystacialPad();

	void createLayer1();
	void createLayer2();
	void createAnchor();
	void createIntrinsicSlingMuscle();
	//void contractIntrinsicSlingMuscle(int step);
	//void contractIntrinsicSlingMuscle(int step, std::vector<int>& those); // spcify which intrinsic muscle to contract by "those"
	void contractIntrinsicSlingMuscle(btScalar ratio);

	void createNasolabialis();
	void contractNasolabialis(btScalar ratio);
	void createMaxillolabialis();
	void contractMaxillolabialis(btScalar ratio);
	void createNasolabialisSuperficialis();
	void createParsMediaSuperior();
	void createParsMediaInferior();
	void createParsInternaProfunda();
	void createParsMaxillaris();


	void update();
	
	int getNumFollicles() const;
	Follicle* getFollicleByIndex(int idx);

	void debugDraw();
	inline btDynamicsWorld* getWorld() {
		return m_sim->getDynamicsWorld();
	}
	inline btAlignedObjectArray<btCollisionShape*>* getCollisionShapes() {
		return &(m_sim->m_collisionShapes);
	}
	
};





#endif
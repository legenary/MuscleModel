#ifndef MYSTACIAL_PAD_H
#define MYSTACIAL_PAD_H

class Follicle;
class Tissue;
class IntrinsicSlingMuscle;
class ExtrinsicMuscle;
class Parameter;

class MystacialPad {
private:
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
	int nTissueISM;
	ExtrinsicMuscle* m_nasolabialis;
	ExtrinsicMuscle* m_maxillolabialis;
	ExtrinsicMuscle* m_NS;
	ExtrinsicMuscle* m_PMS;
	ExtrinsicMuscle* m_PMI;
	ExtrinsicMuscle* m_PIP;
	ExtrinsicMuscle* m_PM;
	std::vector<std::vector<float>> heightPlaceHolder;


public:
	MystacialPad(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	MystacialPad(const MystacialPad&) = delete;
	MystacialPad& operator=(MystacialPad const&) = delete;
	virtual ~MystacialPad();

	void createLayer1(btDiscreteDynamicsWorld* world, Parameter* param);
	void createLayer2(btDiscreteDynamicsWorld* world, Parameter* param);
	void createAnchor(btDiscreteDynamicsWorld* world, Parameter* param);
	void createIntrinsicSlingMuscle(btDiscreteDynamicsWorld* world, Parameter* param);
	void contractIntrinsicSlingMuscle(int step,  Parameter* param);
	void contractIntrinsicSlingMuscle(int step, Parameter* param, std::vector<int>& those); // spcify which intrinsic muscle to contract by "those"
	
	void createNasolabialis(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	void contractNasolabialis(int step, Parameter* param);
	void createMaxillolabialis(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	void contractMaxillolabialis(int step, Parameter* param);
	void createNasolabialisSuperficialis(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	void createParsMediaSuperior(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	void createParsMediaInferior(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	void createParsInternaProfunda(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	void createParsMaxillaris(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);


	void update();
	
	int getNumFollicles() const;
	Follicle* getFollicleByIndex(int idx);

	void debugDraw(btDiscreteDynamicsWorld* world, int DEBUG);
	
};





#endif
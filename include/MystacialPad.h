#ifndef MYSTACIAL_PAD_H
#define MYSTACIAL_PAD_H

#include "Follicle.h"
#include "Utility.h"
#include "Parameter.h"
#include "Spring.h"
#include "IntrinsicMuscle.h"

#include "LinearMath/btAlignedObjectArray.h"

class MystacialPad {
private:
	btAlignedObjectArray<Follicle*> m_follicleArray;
	btAlignedObjectArray<Spring*> m_layer1;
	btAlignedObjectArray<Spring*> m_layer2;
	btAlignedObjectArray<Spring*> m_layer3;
	btAlignedObjectArray<IntrinsicSlingMuscle*> m_intrinsicSlingMuscleArray;
	int nFollicle;
	int nSpringLayer1;
	int nSpringLayer2;
	int nSpringIntrinscisSlingMuscle;


public:
	MystacialPad(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	virtual ~MystacialPad() {}

	void createLayer1(btDiscreteDynamicsWorld* world, Parameter* param);
	void createLayer2(btDiscreteDynamicsWorld* world, Parameter* param);
	void createIntrinsicSlingMuscle(btDiscreteDynamicsWorld* world, Parameter* param);
	void update();
	
	int getNumFollicles() const;
	Follicle* getFollicleByIndex(int idx);

	void debugDraw(btDiscreteDynamicsWorld* world, int DEBUG);
	
};





#endif
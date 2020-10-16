#ifndef MYSTACIAL_PAD_H
#define MYSTACIAL_PAD_H

#include "Follicle.h"
#include "Utility.h"
#include "Parameter.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

class MystacialPad {
private:
	btAlignedObjectArray<Follicle*> m_follicleArray;
	btAlignedObjectArray<btGeneric6DofSpringConstraint*> m_layer1;
	btAlignedObjectArray<btGeneric6DofSpringConstraint*> m_layer2;
	btAlignedObjectArray<btGeneric6DofSpringConstraint*> m_layer3;
	int nFollicle;


public:
	MystacialPad(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	virtual ~MystacialPad() {}

	void createLayer1(btDiscreteDynamicsWorld* world, Parameter* param);
	
	int getNumFollicles();
	Follicle* getFollicleByIndex(int idx);

};





#endif
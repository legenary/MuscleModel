#ifndef MYSTACIAL_PAD_H
#define MYSTACIAL_PAD_H

#include "Follicle.h"
#include "Utility.h"
#include "Parameter.h"
#include "Spring.h"

#include "LinearMath/btAlignedObjectArray.h"

class MystacialPad {
private:
	btAlignedObjectArray<Follicle*> m_follicleArray;
	btAlignedObjectArray<Spring*> m_layer1;
	btAlignedObjectArray<Spring*> m_layer2;
	btAlignedObjectArray<Spring*> m_layer3;
	int nFollicle;
	int nSpringLayer1;
	int nSpringLayer2;


public:
	MystacialPad(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param);
	virtual ~MystacialPad() {}

	void test(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param);
	void createLayer1(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param);
	void createLayer2(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param);
	void update();
	
	int getNumFollicles() const;
	Follicle* getFollicleByIndex(int idx);

	void test();
	
};





#endif
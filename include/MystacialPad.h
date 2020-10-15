#ifndef MYSTACIAL_PAD_H
#define MYSTACIAL_PAD_H

#include "Follicle.h"
#include "Parameter.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

class MystacialPad {
private:
	btAlignedObjectArray<Follicle*> m_follicleArray;
	Parameter* param;


public:
	MystacialPad(){}
	virtual ~MystacialPad() {}
	int nFollicle;




};





#endif
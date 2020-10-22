#ifndef INTRINSIC_MUSCLE_H
#define INTRINSIC_MUSCLE_H

#include "Spring.h"
#include "Utility.h"
#include "Follicle.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Intrinsic sling muscle
class IntrinsicSlingMuscle : public Spring {
private:
	void init();

public:
	IntrinsicSlingMuscle(Follicle* fol1, Follicle* fol2, btTransform frameInParent, btTransform frameInChild, btScalar k);
		
	virtual ~IntrinsicSlingMuscle(){}

	void contract();

};



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Intrinsic oblique muscle
class IntrinsicObliqueMuscle : public Spring {
private:
	void init();

public:
	IntrinsicObliqueMuscle(Follicle* fol1, Follicle* fol2, btTransform frameInParent, btTransform frameInChild, btScalar k);

	virtual ~IntrinsicObliqueMuscle() {}

	void contract();

};



#endif
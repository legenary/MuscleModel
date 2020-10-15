#ifndef PARAMETER_H
#define PARAMETER_H

#include "LinearMath/btVector3.h"

class Parameter {

public:
	Parameter();
	virtual ~Parameter() {}

	btScalar m_time_step;
	btScalar m_num_internal_step;
	btScalar m_time_stop;

	btScalar camPos[3];
	btScalar camDist;
	btScalar camPitch;
	btScalar camYaw;

};



#endif
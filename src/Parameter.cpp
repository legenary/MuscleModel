#include "Parameter.h"

Parameter::Parameter() {
	m_time_step = 1. / 60.;
	m_num_internal_step = 10;
	m_time_stop = 70;


	//camera position
	camPos[0] = 0;
	camPos[1] = 0;
	camPos[2] = 3;
	camDist = 30;
	camPitch = -50;
	camYaw = 0;

}
#include "Parameter.h"

Parameter::Parameter() {
	m_time_step = 1. / 60.;
	m_num_internal_step = 10;
	m_time_stop = 70;


	//camera position
	camPos[0] = 0;
	camPos[1] = 0;
	camPos[2] = 3;
	camDist = 10;
	camPitch = -50;
	camYaw = 0;

	// foillicle parameter
	dir_follicle_loc_orient = "../resources/follicle_position_orientation.csv";
	fol_height = 0.5;
	fol_radius = 0.1;

}
#include "Parameter.h"

Parameter::Parameter() {
	m_time_step = 1./60.;
	m_num_internal_step = 10;
	m_time_stop = 15.;


	//camera position
	camPos[0] = 0;
	camPos[1] = 0;
	camPos[2] = 3;
	camDist = 4;
	camPitch = -50;
	camYaw = 0;

	// foillicle parameter
	dir_follicle_loc_orient = "../resources/follicle_pos_ypr.csv";
	fol_height = 0.5;
	fol_radius = 0.1;

	// layer/spring parameter
	dir_spring_hex_mesh_index = "../resources/spring_hex_mesh_idx.csv";
	k_layer1 = 0.1;
	k_layer2 = 0.1;
}
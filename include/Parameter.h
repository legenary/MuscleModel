#ifndef PARAMETER_H
#define PARAMETER_H

#include "LinearMath/btVector3.h"
#include <vector>
#include <string>

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

	// follicle parameter
	std::string dir_follicle_loc_orient;
	std::vector<std::vector<float>> FOLLICLE_LOC_ORIENT;
	btScalar fol_height;
	btScalar fol_radius;

	// layer/spring parameter
	std::string dir_spring_hex_mesh_index;
	std::vector<std::vector<int>> SPRING_HEX_MESH_INDEX;
	btScalar k_layer1;
	btScalar k_layer2;

};



#endif
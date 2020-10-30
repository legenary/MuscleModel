#ifndef PARAMETER_H
#define PARAMETER_H

#include "Utility.h"

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
	int DEBUG;

	btScalar camPos[3];
	btScalar camDist;
	btScalar camPitch;
	btScalar camYaw;

	// follicle parameter
	std::string dir_follicle_loc_orient;
	std::vector<std::vector<float>> FOLLICLE_LOC_ORIENT;
	btScalar fol_half_height;
	btScalar fol_radius;

	// layer/spring parameter
	std::string dir_spring_hex_mesh_idx;
	std::vector<std::vector<int>> SPRING_HEX_MESH_IDX;
	btScalar k_layer1;
	btScalar k_layer2;
	btScalar damping;
	btScalar k_anchor;

	// instrinsic sling muscle parameter
	std::string dir_intrinsic_sling_muscle_idx;
	std::vector<std::vector<int>>INTRINSIC_SLING_MUSCLE_IDX;
	btScalar k_ISM;
	// contraction
	bool contractISM;
	std::string dir_intrinsic_sling_muscle_contraction_trajectory;
	std::vector<std::vector<float>> INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY;

	// M.nasolabialis
	std::string dir_nasolabialis_node_pos;
	std::vector<std::vector<float>> NASOLABIALIS_NODE_POS;
	std::string dir_nasolabialis_construction_idx;
	std::vector<std::vector<int>> NASOLABIALIS_CONSTRUCTION_IDX;
	std::string dir_nasolabialis_insertion_idx;
	std::vector<std::vector<int>> NASOLABIALIS_INSERTION_IDX;
	btScalar k_nasolabialis;
	// contraction
	std::string dir_nasolabialis_contraction_trajectory;
	std::vector<std::vector<float>> NASOLABIALIS_CONTRACTION_TRAJECTORY;
	bool contractNasolabialis;

	// M.maxillolabialis
	std::string dir_maxillolabialis_node_pos;
	std::vector<std::vector<float>> MAXILLOLABIALIS_NODE_POS;
	std::string dir_maxillolabialis_construction_idx;
	std::vector<std::vector<int>> MAXILLOLABIALIS_CONSTRUCTION_IDX;
	std::string dir_maxillolabialis_insertion_idx;
	std::vector<std::vector<int>> MAXILLOLABIALIS_INSERTION_IDX;
	btScalar k_maxillolabialis;
	// contraction
	std::string dir_maxillolabialis_contraction_trajectory;
	std::vector<std::vector<float>> MAXILLOLABIALIS_CONTRACTION_TRAJECTORY;
	bool contractMaxillolabialis;


};



#endif
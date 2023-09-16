#ifndef PARAMETER_H
#define PARAMETER_H

#include "Utility.h"

enum class MODEL {
	TEST,
	FULL,
	REDUCED
};

enum class BENDING_MODEL {
	SPRING,
	DIHEDRAL_ANGLE
};

enum MUSCLE {
	NONE = 0,
	ISM = BIT(0),
	N = BIT(1),
	M = BIT(2),
	NS = BIT(3),
	PMS = BIT(4),
	PIP = BIT(5),
	PMI = BIT(6),
	PM = BIT(7),
	POO = BIT(9)
};
static std::unordered_map<MUSCLE, std::string> MSUCLEstrings = {
	{MUSCLE::NONE, "NONE"},
	{MUSCLE::ISM, "ISM"},
	{MUSCLE::N, "N"},
	{MUSCLE::M, "M"},
	{MUSCLE::NS, "NS"},
	{MUSCLE::PMS, "PMS"},
	{MUSCLE::PMI, "PMI"},
	{MUSCLE::PIP, "PIP"},
	{MUSCLE::PM, "PM"},
	{MUSCLE::POO, "POO"},
};

class Parameter {

public:
	Parameter();
	virtual ~Parameter() {}

	MODEL m_model;
	MODEL getArrayModel() const { return m_model; }
	BENDING_MODEL m_bending_model;
	BENDING_MODEL getBendingModel() const { return m_bending_model; }


	int m_fps;
	btScalar m_time_step;
	int m_num_internal_step;
	btScalar m_internal_time_step;
	btScalar m_time_stop;
	btScalar inverse_fiber_query_rate;
	int DEBUG;

	uint16_t FlagContractMuscle;
	btScalar contract_to;
	btScalar whisking_frequency;
	btScalar phase1_count;
	btScalar phase2_count;
	btScalar phase1_offset;
	btScalar phase1_peak;
	btScalar phase2_offset;
	btScalar phase2_peak;

	btScalar camPos[3];
	btScalar camDist;
	btScalar camPitch;
	btScalar camYaw;

	// follicle parameter
	std::string dir_follicle_pos_orient_len_vol;
	std::string dir_follicle_pos_orient_len_vol_reduced;
	std::vector<std::vector<float>> FOLLICLE_POS_ORIENT_LEN_VOL;
	btScalar fol_radius;
	btScalar fol_density;
	btScalar fol_damping;

	// layer/anchor spring parameter
	std::string dir_spring_hex_mesh_idx;
	std::string dir_spring_hex_mesh_idx_reduced;
	std::vector<std::vector<int>> SPRING_HEX_MESH_IDX;
	std::string dir_spring_bending_idx;
	std::string dir_spring_bending_idx_reduced;
	std::vector<std::vector<int>> SPRING_BENDING_IDX;

	btScalar k_layer1;
	btScalar k_layer2;
	btScalar zeta_layer;

	btScalar k_anchor_translational;
	btScalar k_anchor_torsional;
	btScalar zeta_anchor_translational;
	btScalar zeta_anchor_torsional;

	uint16_t FlagCreateMuscles;

	btScalar f0;

	// instrinsic sling muscle parameter
	std::string dir_intrinsic_sling_muscle_idx;
	std::string dir_intrinsic_sling_muscle_idx_reduced;
	std::vector<std::vector<int>>INTRINSIC_SLING_MUSCLE_IDX;
	std::string dir_intrinsic_sling_muscle_greek;
	std::string dir_intrinsic_sling_muscle_greek_reduced;
	std::vector<std::vector<float>>INTRINSIC_SLING_MUSCLE_GREEK;
	btScalar f0_ISM;
	// contraction
	std::string dir_intrinsic_sling_muscle_contraction_trajectory;
	std::vector<std::vector<float>> INTRINSIC_SLING_MUSCLE_CONTRACTION_TRAJECTORY;
	btScalar muslce_activation_tau_a;
	btScalar muslce_activation_tau_d;

	// M.nasolabialis
	std::string dir_nasolabialis_node_pos;
	std::string dir_nasolabialis_node_pos_reduced;
	std::vector<std::vector<float>> NASOLABIALIS_NODE_POS;
	std::string dir_nasolabialis_construction_idx;
	std::string dir_nasolabialis_construction_idx_reduced;
	std::vector<std::vector<int>> NASOLABIALIS_CONSTRUCTION_IDX;
	std::string dir_nasolabialis_insertion_idx;
	std::string dir_nasolabialis_insertion_idx_reduced;
	std::vector<std::vector<int>> NASOLABIALIS_INSERTION_IDX;
	btScalar f0_nasolabialis;


	// M.maxillolabialis
	std::string dir_maxillolabialis_node_pos;
	std::string dir_maxillolabialis_node_pos_reduced;
	std::vector<std::vector<float>> MAXILLOLABIALIS_NODE_POS;
	std::string dir_maxillolabialis_construction_idx;
	std::string dir_maxillolabialis_construction_idx_reduced;
	std::vector<std::vector<int>> MAXILLOLABIALIS_CONSTRUCTION_IDX;
	std::string dir_maxillolabialis_insertion_idx;
	std::string dir_maxillolabialis_insertion_idx_reduced;
	std::vector<std::vector<int>> MAXILLOLABIALIS_INSERTION_IDX;
	btScalar f0_maxillolabialis;

	// M. Nasolabialis superficialis
	std::string dir_nasolabialis_superficialis_node_pos;
	std::vector<std::vector<float>> NASOLABIALIS_SUPERFICIALIS_NODE_POS;
	std::string dir_nasolabialis_superficialis_construction_idx;
	std::vector<std::vector<int>> NASOLABIALIS_SUPERFICIALIS_CONSTRUCTION_IDX;
	std::string dir_nasolabialis_superficialis_insertion_idx;
	std::vector<std::vector<int>> NASOLABIALIS_SUPERFICIALIS_INSERTION_IDX;
	btScalar f0_NS;

	// M. Nasolabialis superficialis
	std::string dir_pars_orbicularis_oris_node_pos;
	std::vector<std::vector<float>> PARS_ORBICULARIS_ORIS_NODE_POS;
	std::string dir_pars_orbicularis_oris_construction_idx;
	std::vector<std::vector<int>> PARS_ORBICULARIS_ORIS_CONSTRUCTION_IDX;
	std::string dir_pars_orbicularis_oris_insertion_idx;
	std::vector<std::vector<int>> PARS_ORBICULARIS_ORIS_INSERTION_IDX;
	btScalar f0_POO;

	// Pars media superior of M. Nasolabialis profundus
	std::string dir_pars_media_superior_node_pos;
	std::string dir_pars_media_superior_node_pos_reduced;
	std::vector<std::vector<float>> PARS_MEDIA_SUPERIOR_NODE_POS;
	std::string dir_pars_media_superior_construction_idx;
	std::string dir_pars_media_superior_construction_idx_reduced;
	std::vector<std::vector<int>> PARS_MEDIA_SUPERIOR_CONSTRUCTION_IDX;
	std::string dir_pars_media_superior_insertion_idx;
	std::string dir_pars_media_superior_insertion_idx_reduced;
	std::vector<std::vector<int>> PARS_MEDIA_SUPERIOR_INSERTION_IDX;
	std::string dir_pars_media_superior_insertion_height;
	std::string dir_pars_media_superior_insertion_height_reduced;
	std::vector<std::vector<float>> PARS_MEDIA_SUPERIOR_INSERTION_HEIGHT;
	btScalar f0_PMS;

	// Pars interna profunda of M. Nasolabialis profundus
	std::string dir_pars_interna_profunda_node_pos;
	std::string dir_pars_interna_profunda_node_pos_reduced;
	std::vector<std::vector<float>> PARS_INTERNA_PROFUNDA_NODE_POS;
	std::string dir_pars_interna_profunda_construction_idx;
	std::string dir_pars_interna_profunda_construction_idx_reduced;
	std::vector<std::vector<int>> PARS_INTERNA_PROFUNDA_CONSTRUCTION_IDX;
	std::string dir_pars_interna_profunda_insertion_idx;
	std::string dir_pars_interna_profunda_insertion_idx_reduced;
	std::vector<std::vector<int>> PARS_INTERNA_PROFUNDA_INSERTION_IDX;
	std::string dir_pars_interna_profunda_insertion_height;
	std::string dir_pars_interna_profunda_insertion_height_reduced;
	std::vector<std::vector<float>> PARS_INTERNA_PROFUNDA_INSERTION_HEIGHT;
	btScalar f0_PIP;

	// Pars media inferior of M. Nasolabialis profundus
	std::string dir_pars_media_inferior_node_pos;
	std::string dir_pars_media_inferior_node_pos_reduced;
	std::vector<std::vector<float>> PARS_MEDIA_INFERIOR_NODE_POS;
	std::string dir_pars_media_inferior_construction_idx;
	std::string dir_pars_media_inferior_construction_idx_reduced;
	std::vector<std::vector<int>> PARS_MEDIA_INFERIOR_CONSTRUCTION_IDX;
	std::string dir_pars_media_inferior_insertion_idx;
	std::string dir_pars_media_inferior_insertion_idx_reduced;
	std::vector<std::vector<int>> PARS_MEDIA_INFERIOR_INSERTION_IDX;
	std::string dir_pars_media_inferior_insertion_height;
	std::string dir_pars_media_inferior_insertion_height_reduced;
	std::vector<std::vector<float>> PARS_MEDIA_INFERIOR_INSERTION_HEIGHT;
	btScalar f0_PMI;

	// Pars maxillaris superficialis of M. Nasolabialis profundus
	// Pars maxillaris profunda of M. Nasolabialis profundus
	std::string dir_pars_maxillaris_node_pos;
	std::string dir_pars_maxillaris_node_pos_reduced;
	std::vector<std::vector<float>> PARS_MAXILLARIS_NODE_POS;
	std::string dir_pars_maxillaris_construction_idx;
	std::string dir_pars_maxillaris_construction_idx_reduced;
	std::vector<std::vector<int>> PARS_MAXILLARIS_CONSTRUCTION_IDX;
	std::string dir_pars_maxillaris_insertion_idx;
	std::string dir_pars_maxillaris_insertion_idx_reduced;
	std::vector<std::vector<int>> PARS_MAXILLARIS_INSERTION_IDX;
	std::string dir_pars_maxillaris_insertion_height;
	std::string dir_pars_maxillaris_insertion_height_reduced;
	std::vector<std::vector<float>> PARS_MAXILLARIS_INSERTION_HEIGHT;
	btScalar f0_PM;

	// output
	bool VIDEO;
	char video_file_name[64];
	bool OUTPUT;
	char output_path[64];

	// getters
	inline int getFPS() const { return m_fps; }
};



#endif
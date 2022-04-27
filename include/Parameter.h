#ifndef PARAMETER_H
#define PARAMETER_H

class Parameter {

public:
	Parameter();
	virtual ~Parameter() {}

	int m_fps;
	btScalar m_time_step;
	btScalar m_num_internal_step;
	btScalar m_internal_time_step;
	btScalar m_time_stop;
	int DEBUG;

	btScalar camPos[3];
	btScalar camDist;
	btScalar camPitch;
	btScalar camYaw;

	// follicle parameter
	std::string dir_follicle_pos_orient_len_vol;
	std::vector<std::vector<float>> FOLLICLE_POS_ORIENT_LEN_VOL;
	btScalar fol_radius;
	btScalar fol_density;

	// layer/spring parameter
	std::string dir_spring_hex_mesh_idx;
	std::vector<std::vector<int>> SPRING_HEX_MESH_IDX;
	btScalar E_skin;
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

	// M. Nasolabialis superficialis
	std::string dir_nasolabialis_superficialis_node_pos;
	std::vector<std::vector<float>> NASOLABIALIS_SUPERFICIALIS_NODE_POS;
	std::string dir_nasolabialis_superficialis_construction_idx;
	std::vector<std::vector<int>> NASOLABIALIS_SUPERFICIALIS_CONSTRUCTION_IDX;
	std::string dir_nasolabialis_superficialis_insertion_idx;
	std::vector<std::vector<int>> NASOLABIALIS_SUPERFICIALIS_INSERTION_IDX;
	btScalar k_NS;

	// Pars media superior of M. Nasolabialis profundus
	std::string dir_pars_media_superior_node_pos;
	std::vector<std::vector<float>> PARS_MEDIA_SUPERIOR_NODE_POS;
	std::string dir_pars_media_superior_construction_idx;
	std::vector<std::vector<int>> PARS_MEDIA_SUPERIOR_CONSTRUCTION_IDX;
	std::string dir_pars_media_superior_insertion_idx;
	std::vector<std::vector<int>> PARS_MEDIA_SUPERIOR_INSERTION_IDX;
	std::string dir_pars_media_superior_insertion_height;
	std::vector<std::vector<float>> PARS_MEDIA_SUPERIOR_INSERTION_HEIGHT;
	btScalar k_PMS;

	// Pars media inferior of M. Nasolabialis profundus
	std::string dir_pars_media_inferior_node_pos;
	std::vector<std::vector<float>> PARS_MEDIA_INFERIOR_NODE_POS;
	std::string dir_pars_media_inferior_construction_idx;
	std::vector<std::vector<int>> PARS_MEDIA_INFERIOR_CONSTRUCTION_IDX;
	std::string dir_pars_media_inferior_insertion_idx;
	std::vector<std::vector<int>> PARS_MEDIA_INFERIOR_INSERTION_IDX;
	std::string dir_pars_media_inferior_insertion_height;
	std::vector<std::vector<float>> PARS_MEDIA_INFERIOR_INSERTION_HEIGHT;
	btScalar k_PMI;

	// Pars interna profunda of M. Nasolabialis profundus
	std::string dir_pars_interna_profunda_node_pos;
	std::vector<std::vector<float>> PARS_INTERNA_PROFUNDA_NODE_POS;
	std::string dir_pars_interna_profunda_construction_idx;
	std::vector<std::vector<int>> PARS_INTERNA_PROFUNDA_CONSTRUCTION_IDX;
	std::string dir_pars_interna_profunda_insertion_idx;
	std::vector<std::vector<int>> PARS_INTERNA_PROFUNDA_INSERTION_IDX;
	std::string dir_pars_interna_profunda_insertion_height;
	std::vector<std::vector<float>> PARS_INTERNA_PROFUNDA_INSERTION_HEIGHT;
	btScalar k_PIP;

	// Pars maxillaris superficialis of M. Nasolabialis profundus
	// Pars maxillaris profunda of M. Nasolabialis profundus
	std::string dir_pars_maxillaris_node_pos;
	std::vector<std::vector<float>> PARS_MAXILLARIS_NODE_POS;
	std::string dir_pars_maxillaris_construction_idx;
	std::vector<std::vector<int>> PARS_MAXILLARIS_CONSTRUCTION_IDX;
	std::string dir_pars_maxillaris_insertion_idx;
	std::vector<std::vector<int>> PARS_MAXILLARIS_INSERTION_IDX;
	std::string dir_pars_maxillaris_insertion_height;
	std::vector<std::vector<float>> PARS_MAXILLARIS_INSERTION_HEIGHT;
	btScalar k_PM;

	// getter
	inline int getFPS() { return m_fps; }

};



#endif
#include "Parameter.h"

Parameter::Parameter() {
	m_time_step = 1./60.;
	m_num_internal_step = 10;
	m_time_stop = 0;

	DEBUG = 1;	// 0: no debug
				// 1: specified inline debug drawing only
				// 2: wire frame added
				// 3: axis aligned bound box added

	//camera position
	camPos[0] = 0;
	camPos[1] = 0;
	camPos[2] = 0;
	camDist = 10;
	camPitch = -89;
	camYaw = 0;

	// foillicle parameter
	dir_follicle_loc_orient = "../resources/follicle_pos_ypr.csv";
	fol_half_height = 0.7;
	fol_radius = 0.1;

	// layer/spring parameter
	dir_spring_hex_mesh_idx = "../resources/spring_hex_mesh_idx.csv";
	k_layer1 = 1000;
	k_layer2 = 1000;
	damping = 0.001;	// still need some work to find what critical damping ratio is
	k_anchor = 500;		// k = 0 : hard anchor, no linear displacement, free angular movement
						// k > 0 : soft anchor, springy linear and angular movement

	// instrinsic sling muscle parameter
	dir_intrinsic_sling_muscle_idx = "../resources/intrinsic_sling_muscle_idx.csv";
	k_ISM = 2000;
	// contraction
	dir_intrinsic_sling_muscle_contraction_trajectory = "../resources/intrinsic_sling_muscle_contraction_trajectory.csv";
	contractISM = true;

	// M.Nasolabialis
	dir_nasolabialis_node_pos = "../resources/nasolabialis_node_pos.csv";
	dir_nasolabialis_construction_idx = "../resources/nasolabialis_construction_idx.csv";
	dir_nasolabialis_insertion_idx = "../resources/nasolabialis_insertion_idx.csv";
	k_nasolabialis = 500;
	// contraction
	dir_nasolabialis_contraction_trajectory = "../resources/nasolabialis_contraction_trajectory.csv";
	contractNasolabialis = false;

	// M.Maxillolabialis
	dir_maxillolabialis_node_pos = "../resources/maxillolabialis_node_pos.csv";
	dir_maxillolabialis_construction_idx = "../resources/maxillolabialis_construction_idx.csv";
	dir_maxillolabialis_insertion_idx = "../resources/maxillolabialis_insertion_idx.csv";
	k_maxillolabialis = 500;
	// contraction
	dir_maxillolabialis_contraction_trajectory = "../resources/maxillolabialis_contraction_trajectory.csv";
	contractMaxillolabialis = false;

	// Pars media superior of M. Nasolabialis profundus
	dir_pars_media_superior_node_pos = "../resources/pars_media_superior_node_pos.csv";
	dir_pars_media_superior_construction_idx = "../resources/pars_media_superior_construction_idx.csv";
	dir_pars_media_superior_insertion_idx = "../resources/pars_media_superior_insertion_idx.csv";
	dir_pars_media_superior_insertion_height = "../resources/pars_media_superior_insertion_height.csv";
	k_PMS = 500;

	// Pars media inferior of M. Nasolabialis profundus
	dir_pars_media_inferior_node_pos = "../resources/pars_media_inferior_node_pos.csv";
	dir_pars_media_inferior_construction_idx = "../resources/pars_media_inferior_construction_idx.csv";
	dir_pars_media_inferior_insertion_idx = "../resources/pars_media_inferior_insertion_idx.csv";
	dir_pars_media_inferior_insertion_height = "../resources/pars_media_inferior_insertion_height.csv";
	k_PMI = 500;

	// Pars interna profunda of M.Nasolabialis profundus
	dir_pars_interna_profunda_node_pos = "../resources/pars_interna_profunda_node_pos.csv";
	dir_pars_interna_profunda_construction_idx = "../resources/pars_interna_profunda_construction_idx.csv";
	dir_pars_interna_profunda_insertion_idx = "../resources/pars_interna_profunda_insertion_idx.csv";
	dir_pars_interna_profunda_insertion_height = "../resources/pars_interna_profunda_insertion_height.csv";
	k_PIP = 500;

	// Pars maxillaris superficialis of M. Nasolabialis profundus
	// Pars maxillaris profunda of M. Nasolabialis profundus
	dir_pars_maxillaris_node_pos = "../resources/pars_maxillaris_node_pos.csv";
	dir_pars_maxillaris_construction_idx = "../resources/pars_maxillaris_construction_idx.csv";
	dir_pars_maxillaris_insertion_idx = "../resources/pars_maxillaris_insertion_idx.csv";
	dir_pars_maxillaris_insertion_height = "../resources/pars_maxillaris_insertion_height.csv";
	k_PM = 500;

}
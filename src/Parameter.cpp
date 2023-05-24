#include "my_pch.h"
#include "Parameter.h"

#include "Utility.h"

Parameter::Parameter() {
	m_fps = 120;
	m_time_step = 1.0f / m_fps;
	m_num_internal_step = 30;	// for constraint solver
								// Note: number of iterations grows linear with mass ratios
								// iter = 3*ratio + 2
	m_internal_time_step = m_time_step / (btScalar) m_num_internal_step;
	m_time_stop = 1;

	inverse_fiber_query_rate = 1.0f / 60.0f;

	DEBUG = 1;	// 0: no debug
				// 1: specified inline debug drawing only
				// 2: wire frame added
				// 3: axis aligned bound box added

	// contract
	contract_range = 0.25;
	contract_frequency = 1; // Hz
	
	//camera position
	camPos[0] = 0;
	camPos[1] = 0;
	camPos[2] = 0;
	camDist = 8;
	camPitch = -89;
	camYaw = 0;

	// foillicle parameter
	dir_follicle_pos_orient_len_vol = "../resources/follicle_pos_ypr_len_vol.csv";
	dir_follicle_pos_orient_len_vol_reduced = "../resources/follicle_pos_ypr_len_vol_reduced.csv";
	fol_radius = 0.15;		// Bullet unit: mm
	fol_density = 0.001;	// Bullet unit: g/mm^3
	fol_damping = 1;		// damping for rigid body is clamped between 0 and 1
							// default: 0, no damping
							// dampnig is implemented in tissue
	fol_idx_reduced = { 12, 13, 19, 20 };


	// layer tissue parameter
	dir_spring_hex_mesh_idx = "../resources/spring_hex_mesh_idx.csv";
	dir_spring_hex_mesh_idx_reduced = "../resources/spring_hex_mesh_idx_reduced.csv";
	E_skin = 8000000;		// unit: Pa, do not change
							// Skin's Young's Modulus is ~8MPa (Karimi and Navidbakhsh, 2015)

	k_layer1 = 250;			// Bullet unit: 1e-3 (N/m)
	k_layer2 = 500;
	k_anchor = 250;			// k > 0 : soft anchor, springy linear and angular movement
							// k = 0 : hard anchor, no linear displacement, free angular movement

	zeta_tissue = 10;		// This sets the damping ratio for: 
							// (1) tissue anchor,
							// (2) extrinsic muscle anchor
							// (3) extrinsic muscle insertion (this is treated as tissue)
							// F = -k * x - c* v
							// where c = zeta * (2 * sqrt(m * k))
							// zeta > 1: overdamped
							// zeta = 1: critically damped
							// zeta < 1: underdamped
							// reference value: 0.01

	// muscle parameter
	btScalar f0 = 8;		// Bullet unit: 1e-6 (N), uN
	// instrinsic sling muscle parameter
	dir_intrinsic_sling_muscle_idx = "../resources/intrinsic_sling_muscle_idx.csv";
	dir_intrinsic_sling_muscle_idx_reduced = "../resources/intrinsic_sling_muscle_idx_reduced.csv";
	dir_intrinsic_sling_muscle_greek = "../resources/intrinsic_sling_muscle_greek.csv";
	dir_intrinsic_sling_muscle_greek_reduced = "../resources/intrinsic_sling_muscle_greek_reduced.csv";
	f0_ISM = 20*f0; 

	// M.Nasolabialis
	dir_nasolabialis_node_pos = "../resources/nasolabialis_node_pos.csv";
	dir_nasolabialis_node_pos_reduced = "../resources/nasolabialis_node_pos_reduced.csv";
	dir_nasolabialis_construction_idx = "../resources/nasolabialis_construction_idx.csv";
	dir_nasolabialis_construction_idx_reduced = "../resources/nasolabialis_construction_idx_reduced.csv";
	dir_nasolabialis_insertion_idx = "../resources/nasolabialis_insertion_idx.csv";
	dir_nasolabialis_insertion_idx_reduced = "../resources/nasolabialis_insertion_idx_reduced.csv";
	f0_nasolabialis = 25*f0;

	// M.Maxillolabialis
	dir_maxillolabialis_node_pos = "../resources/maxillolabialis_node_pos.csv";
	dir_maxillolabialis_node_pos_reduced = "../resources/maxillolabialis_node_pos_reduced.csv";
	dir_maxillolabialis_construction_idx = "../resources/maxillolabialis_construction_idx.csv";
	dir_maxillolabialis_construction_idx_reduced = "../resources/maxillolabialis_construction_idx_reduced.csv";
	dir_maxillolabialis_insertion_idx = "../resources/maxillolabialis_insertion_idx.csv";
	dir_maxillolabialis_insertion_idx_reduced = "../resources/maxillolabialis_insertion_idx_reduced.csv";
	f0_maxillolabialis = 25*f0;

	// M.Nasolabialis superficialis
	dir_nasolabialis_superficialis_node_pos = "../resources/nasolabialis_superficialis_node_pos.csv";
	dir_nasolabialis_superficialis_construction_idx = "../resources/nasolabialis_superficialis_construction_idx.csv";
	dir_nasolabialis_superficialis_insertion_idx = "../resources/nasolabialis_superficialis_insertion_idx.csv";
	f0_NS = 1*f0;

	// Pars media superior of M. Nasolabialis profundus
	dir_pars_media_superior_node_pos = "../resources/pars_media_superior_node_pos.csv";
	dir_pars_media_superior_construction_idx = "../resources/pars_media_superior_construction_idx.csv";
	dir_pars_media_superior_insertion_idx = "../resources/pars_media_superior_insertion_idx.csv";
	dir_pars_media_superior_insertion_height = "../resources/pars_media_superior_insertion_height.csv";
	f0_PMS = 1*f0;

	// Pars media inferior of M. Nasolabialis profundus
	dir_pars_media_inferior_node_pos = "../resources/pars_media_inferior_node_pos.csv";
	dir_pars_media_inferior_construction_idx = "../resources/pars_media_inferior_construction_idx.csv";
	dir_pars_media_inferior_insertion_idx = "../resources/pars_media_inferior_insertion_idx.csv";
	dir_pars_media_inferior_insertion_height = "../resources/pars_media_inferior_insertion_height.csv";
	f0_PMI = 1*f0;

	// Pars interna profunda of M.Nasolabialis profundus
	dir_pars_interna_profunda_node_pos = "../resources/pars_interna_profunda_node_pos.csv";
	dir_pars_interna_profunda_construction_idx = "../resources/pars_interna_profunda_construction_idx.csv";
	dir_pars_interna_profunda_insertion_idx = "../resources/pars_interna_profunda_insertion_idx.csv";
	dir_pars_interna_profunda_insertion_height = "../resources/pars_interna_profunda_insertion_height.csv";
	f0_PIP = 4*f0;

	// Pars maxillaris superficialis of M. Nasolabialis profundus
	// Pars maxillaris profunda of M. Nasolabialis profundus
	dir_pars_maxillaris_node_pos = "../resources/pars_maxillaris_node_pos.csv";
	dir_pars_maxillaris_construction_idx = "../resources/pars_maxillaris_construction_idx.csv";
	dir_pars_maxillaris_insertion_idx = "../resources/pars_maxillaris_insertion_idx.csv";
	dir_pars_maxillaris_insertion_height = "../resources/pars_maxillaris_insertion_height.csv";
	f0_PM = 4*f0;

	// output;
	VIDEO = true;
	video_file_name = "../output/bundle/output_video.mp4";
	OUTPUT = true;
	output_path = "../output/bundle/";

	// mode
	m_mode = MODE::REDUCED;
}
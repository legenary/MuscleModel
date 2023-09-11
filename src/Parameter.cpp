#include "my_pch.h"
#include "Parameter.h"

Parameter::Parameter() {
	m_fps = 120;
	m_time_step = btScalar(1.0f) / m_fps;
	m_time_stop = 5.0f;

	// fiber query rate:
	// Essentially, we need this rate so the muscle update frequency is fixed, and does not change with simulation FPS
	// Because our muscle dynamics is interpolated, analogous to explicit forward Euler integration, different time step
	// will cause inconsistent behavior. For more details, check out the modeling_log ppt
	inverse_fiber_query_rate = btScalar(1.0f) / 120.0f;
	// Under the current simulation architecture, the muscle should be queried less frequently than main loop update
	ensure(inverse_fiber_query_rate >= m_time_step);

	DEBUG = 1;	// 0: no debug draw
				// 1: specified inline debug drawing only
				// 2: wire frame added
				// 3: axis aligned bound box added
	
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
	fol_radius = 0.32;		// Bullet unit: mm, value estimated from ARP
	fol_density = 0.001;	// Bullet unit: g/mm^3

	// mode
	m_model = MODEL::FULL;
	//FlagCreateMuscles = MUSCLE::ISM | MUSCLE::N | MUSCLE::M;
	FlagCreateMuscles = MUSCLE::ISM | MUSCLE::N | MUSCLE::M | MUSCLE::PIP | MUSCLE::PM | MUSCLE::PMI | MUSCLE::PMS | MUSCLE::NS;

	// contract
	//FlagContractMuscle = MUSCLE::NONE;
	FlagContractMuscle = MUSCLE::ISM;
	//FlagContractMuscle = MUSCLE::N | MUSCLE::M;
	//FlagContractMuscle = MUSCLE::PIP | MUSCLE::PM;
	//FlagContractMuscle = MUSCLE::ISM | MUSCLE::N | MUSCLE::M;
	//FlagContractMuscle = MUSCLE::N | MUSCLE::M | MUSCLE::PIP | MUSCLE::PM;
	//FlagContractMuscle = MUSCLE::ISM | MUSCLE::N | MUSCLE::M | MUSCLE::PIP | MUSCLE::PM;
	//FlagContractMuscle = MUSCLE::PMS | MUSCLE::PMI;

	contract_range = 0.3;
	contract_frequency = 1; // Hz
	contract_count = 3;
	muslce_activation_tau = 0.02; // activation time constant

	switch (m_model) {
	case MODEL::FULL: {
		m_num_internal_step = 60;	// for constraint solver
		// Note: number of iterations grows linear with mass ratios
		// iter = 3*ratio + 2
		// reference value: 40 for reduced-size array, 100 for full-size array
		m_internal_time_step = m_time_step / (btScalar)m_num_internal_step;
		m_bending_model = BENDING_MODEL::SPRING;
		// layer tissue parameter (only translational)
		k_layer1 = 25;			// Bullet unit: 1e-3 (N/m)
		k_layer2 = 50;
		zeta_layer = 1.0;		// This sets the damping ratio for: 
								// (1) two layers
								// (2) extrinsic muscle insertion (this is treated as tissue)
								// F = -k * x - c* v
								// where c = zeta * (2 * sqrt(m * k))
								// zeta > 1: overdamped
								// zeta = 1: critically damped
								// zeta < 1: underdamped

		k_anchor_translational = 10;
		k_anchor_torsional = 20;
								// k > 0 : soft anchor, springy linear and angular movement
								// k = 0 : hard anchor, no linear displacement, free angular movement

		zeta_anchor_translational = 5.5;
		zeta_anchor_torsional = 7.0;		
								// This sets the damping ratio for: 
								// (1) top layer anchor
								// (2) extrinsic muscle originate anchor

		fol_damping = 0;		// damping for rigid body is clamped between 0 and 1
								// default: 0, no damping
								// dampnig is implemented in tissue
		// muscle parameter
		btScalar f0 = 1.5;		// Bullet unit: 1e-6 (N), uN
		f0_ISM = 20*f0; 
		f0_nasolabialis = 25*f0;
		f0_maxillolabialis = 25*f0;
		f0_NS = 1*f0;
		f0_PMS = 1*f0;
		f0_PIP = 4*f0;
		f0_PMI = 1*f0;
		f0_PM = 4*f0;

		// old parameters
		//k_layer1 = 250;			
		//k_layer2 = 500;
		//k_anchor = 250;			
		//zeta_tissue = 10;	
		//fol_damping = 0;
		//btScalar f0 = 8;	
		//f0_ISM = 20 * f0;
		//f0_nasolabialis = 25 * f0;
		//f0_maxillolabialis = 25 * f0;
		//f0_NS = 1 * f0;
		//f0_PMS = 1 * f0;
		//f0_PIP = 4 * f0;
		//f0_PMI = 1 * f0;
		//f0_PM = 4 * f0;
		break;
	}
	case MODEL::TEST:
	case MODEL::REDUCED: {
		m_num_internal_step = 40;
		m_internal_time_step = m_time_step / (btScalar)m_num_internal_step;
		m_bending_model = BENDING_MODEL::DIHEDRAL_ANGLE;
		// In general, a zeta value = 1 means critically damped. But the system is a highly complex system, so damping should be higher to absorb coupled oscillation
		// layer tissue parameter (only translational, no torsional implemented)
		k_layer1 = 25;						// Bullet unit: 1e-3 (N/m) === uN/mm
		k_layer2 = 50;
		zeta_layer = 1.0;					// chosen such that when pulled by M and N, oscillation is minimized
		// anchor parameter (translational and torsional)
		k_anchor_translational = 10;		
		zeta_anchor_translational = 5.5;	// chosen such that a displaced pad (by M and N) is restored to the same location in the same relaxation duration as contraction. Reference 5.5
		k_anchor_torsional = 20;			// chosen to be big enough to bring the pad back to original position in swift time. Reference 20
		zeta_anchor_torsional = 7.0;		// chosen such that a contracted pad (by ISM) is restored to the same location in the same relaxation duration as contraction. Reference 7.0

		fol_damping = 0.0;

		// muscle parameter reduced
		btScalar f0 = 0.6;		// Bullet unit: 1e-6 (N), uN // 0.3 will have ~30 degrees protraction, ISM shortened to 85%
								// chosen such that 
		f0_ISM = 20 * f0;
		f0_nasolabialis = 25 * f0;
		f0_maxillolabialis = 25 * f0;
		f0_NS = 1 * f0;
		f0_PMS = 1 * f0;
		f0_PIP = 4 * f0; // red lower 
		f0_PMI = 1 * f0;
		f0_PM = 4 * f0; // yellow lower
		break;
	}
	}


	// Layers mesh connection
	dir_spring_hex_mesh_idx = "../resources/spring_hex_mesh_idx.csv";
	dir_spring_hex_mesh_idx_reduced = "../resources/spring_hex_mesh_idx_reduced.csv";
	dir_spring_bending_idx = "../resources/spring_bending_idx.csv";
	dir_spring_bending_idx_reduced = "../resources/spring_bending_idx_reduced.csv";

	// Instrinsic Sling Muscle
	dir_intrinsic_sling_muscle_idx = "../resources/intrinsic_sling_muscle_idx.csv";
	dir_intrinsic_sling_muscle_idx_reduced = "../resources/intrinsic_sling_muscle_idx_reduced.csv";
	dir_intrinsic_sling_muscle_greek = "../resources/intrinsic_sling_muscle_greek.csv";
	dir_intrinsic_sling_muscle_greek_reduced = "../resources/intrinsic_sling_muscle_greek_reduced.csv";

	// M.Nasolabialis
	dir_nasolabialis_node_pos = "../resources/nasolabialis_node_pos.csv";
	dir_nasolabialis_node_pos_reduced = "../resources/nasolabialis_node_pos_reduced.csv";
	dir_nasolabialis_construction_idx = "../resources/nasolabialis_construction_idx.csv";
	dir_nasolabialis_construction_idx_reduced = "../resources/nasolabialis_construction_idx_reduced.csv";
	dir_nasolabialis_insertion_idx = "../resources/nasolabialis_insertion_idx.csv";
	dir_nasolabialis_insertion_idx_reduced = "../resources/nasolabialis_insertion_idx_reduced.csv";

	// M.Maxillolabialis
	dir_maxillolabialis_node_pos = "../resources/maxillolabialis_node_pos.csv";
	dir_maxillolabialis_node_pos_reduced = "../resources/maxillolabialis_node_pos_reduced.csv";
	dir_maxillolabialis_construction_idx = "../resources/maxillolabialis_construction_idx.csv";
	dir_maxillolabialis_construction_idx_reduced = "../resources/maxillolabialis_construction_idx_reduced.csv";
	dir_maxillolabialis_insertion_idx = "../resources/maxillolabialis_insertion_idx.csv";
	dir_maxillolabialis_insertion_idx_reduced = "../resources/maxillolabialis_insertion_idx_reduced.csv";

	// M.Nasolabialis superficialis
	dir_nasolabialis_superficialis_node_pos = "../resources/nasolabialis_superficialis_node_pos.csv";
	dir_nasolabialis_superficialis_construction_idx = "../resources/nasolabialis_superficialis_construction_idx.csv";
	dir_nasolabialis_superficialis_insertion_idx = "../resources/nasolabialis_superficialis_insertion_idx.csv";

	// Pars media superior of M. Nasolabialis profundus
	dir_pars_media_superior_node_pos = "../resources/pars_media_superior_node_pos.csv";
	dir_pars_media_superior_node_pos_reduced = "../resources/pars_media_superior_node_pos_reduced.csv";
	dir_pars_media_superior_construction_idx = "../resources/pars_media_superior_construction_idx.csv";
	dir_pars_media_superior_construction_idx_reduced = "../resources/pars_media_superior_construction_idx_reduced.csv";
	dir_pars_media_superior_insertion_idx = "../resources/pars_media_superior_insertion_idx.csv";
	dir_pars_media_superior_insertion_idx_reduced = "../resources/pars_media_superior_insertion_idx_reduced.csv";
	dir_pars_media_superior_insertion_height = "../resources/pars_media_superior_insertion_height.csv";
	dir_pars_media_superior_insertion_height_reduced = "../resources/pars_media_superior_insertion_height_reduced.csv";

	// Pars interna profunda of M.Nasolabialis profundus
	dir_pars_interna_profunda_node_pos = "../resources/pars_interna_profunda_node_pos.csv";
	dir_pars_interna_profunda_node_pos_reduced = "../resources/pars_interna_profunda_node_pos_reduced.csv";
	dir_pars_interna_profunda_construction_idx = "../resources/pars_interna_profunda_construction_idx.csv";
	dir_pars_interna_profunda_construction_idx_reduced = "../resources/pars_interna_profunda_construction_idx_reduced.csv";
	dir_pars_interna_profunda_insertion_idx = "../resources/pars_interna_profunda_insertion_idx.csv";
	dir_pars_interna_profunda_insertion_idx_reduced = "../resources/pars_interna_profunda_insertion_idx_reduced.csv";
	dir_pars_interna_profunda_insertion_height = "../resources/pars_interna_profunda_insertion_height.csv";
	dir_pars_interna_profunda_insertion_height_reduced = "../resources/pars_interna_profunda_insertion_height_reduced.csv";

	// Pars media inferior of M. Nasolabialis profundus
	dir_pars_media_inferior_node_pos = "../resources/pars_media_inferior_node_pos.csv";
	dir_pars_media_inferior_node_pos_reduced = "../resources/pars_media_inferior_node_pos_reduced.csv";
	dir_pars_media_inferior_construction_idx = "../resources/pars_media_inferior_construction_idx.csv";
	dir_pars_media_inferior_construction_idx_reduced = "../resources/pars_media_inferior_construction_idx_reduced.csv";
	dir_pars_media_inferior_insertion_idx = "../resources/pars_media_inferior_insertion_idx.csv";
	dir_pars_media_inferior_insertion_idx_reduced = "../resources/pars_media_inferior_insertion_idx_reduced.csv";
	dir_pars_media_inferior_insertion_height = "../resources/pars_media_inferior_insertion_height.csv";
	dir_pars_media_inferior_insertion_height_reduced = "../resources/pars_media_inferior_insertion_height_reduced.csv";

	// Pars maxillaris superficialis of M. Nasolabialis profundus
	// Pars maxillaris profunda of M. Nasolabialis profundus
	dir_pars_maxillaris_node_pos = "../resources/pars_maxillaris_node_pos.csv";
	dir_pars_maxillaris_node_pos_reduced = "../resources/pars_maxillaris_node_pos_reduced.csv";
	dir_pars_maxillaris_construction_idx = "../resources/pars_maxillaris_construction_idx.csv";
	dir_pars_maxillaris_construction_idx_reduced = "../resources/pars_maxillaris_construction_idx_reduced.csv";
	dir_pars_maxillaris_insertion_idx = "../resources/pars_maxillaris_insertion_idx.csv";
	dir_pars_maxillaris_insertion_idx_reduced = "../resources/pars_maxillaris_insertion_idx_reduced.csv";
	dir_pars_maxillaris_insertion_height = "../resources/pars_maxillaris_insertion_height.csv";
	dir_pars_maxillaris_insertion_height_reduced = "../resources/pars_maxillaris_insertion_height_reduced.csv";

	// output;
	VIDEO = true;
	sprintf(video_file_name, "../output/bundle%s/output_video.mp4", m_model == MODEL::REDUCED ? "_reduced" : "_full");
	OUTPUT = true;
	//output_path = "../output/bundle/";
	sprintf(output_path, "../output/bundle%s/", m_model == MODEL::REDUCED ? "_reduced" : "_full");
}
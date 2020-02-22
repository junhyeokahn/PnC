#include "gtest/gtest.h"
#include <PnC/DracoPnC/PredictionModule/WalkingReferenceTrajectoryModule.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

TEST(ReferenceTest, no_footsteps){
	std::vector<int> index_to_side = {DRACO_RIGHT_FOOTSTEP, DRACO_RIGHT_FOOTSTEP,
									  DRACO_LEFT_FOOTSTEP, DRACO_LEFT_FOOTSTEP};


	WalkingReferenceTrajectoryModule walking_reference_module(index_to_side);

	// Initialize Necessary States
	// CoM Height
	Eigen::Vector3d x_com_pos_in; x_com_pos_in.setZero();
	x_com_pos_in[2] = 0.7;
	// Body Ori
	Eigen::Quaterniond x_ori_start_in; x_ori_start_in.setIdentity();
	// Footstep locations
	DracoFootstep lf_start, rf_start;
    double nominal_width = 0.333657;  // 33.3cm distance between left and right feet
	lf_start.setPosOriSide(Eigen::Vector3d(0.0, nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_LEFT_FOOTSTEP);
	rf_start.setPosOriSide(Eigen::Vector3d(0.0, -nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_RIGHT_FOOTSTEP);



	// Initialize Footsteps
	Eigen::Vector3d foot_translate(0.25, 0.1, 0.0);
	Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(-M_PI/12.0, Eigen::Vector3d::UnitZ()) );
	myUtils::pretty_print(foot_rotate, std::cout, "quat_foot_rotate");

	// stance foot is the left foot intially. So let us take a right footstep forward.
	DracoFootstep right_footstep1;
	right_footstep1.setPosOriSide(rf_start.position + foot_translate, foot_rotate*rf_start.orientation, DRACO_RIGHT_FOOTSTEP);

	DracoFootstep left_footstep1;
	left_footstep1.setPosOriSide(lf_start.position + foot_translate, lf_start.orientation, DRACO_LEFT_FOOTSTEP);

	std::cout << "rf step 1" << std::endl;
	right_footstep1.printInfo();
	std::cout << "lf step 1" << std::endl;
	left_footstep1.printInfo();


	// left then right footstep
	std::vector<DracoFootstep> footstep_list;

	double t_walk_start = 0.5;
 	walking_reference_module.setStartingConfiguration(x_com_pos_in,
													  x_ori_start_in,
								  					  lf_start,
								  					  rf_start);
	walking_reference_module.setFootsteps(t_walk_start, footstep_list);


	walking_reference_module.walking_rfs_ptr->testFunction();


	double t_query = 0.0;
	double t_total = 2.5;
	double dt = 0.01;
	int n_total = static_cast<int>(t_total/dt);

	int state;
	Eigen::Vector3d x_com_ref;
	Eigen::Quaterniond x_ori_ref;
	Eigen::VectorXd max_z_force(index_to_side.size());

	DracoFootstep mpc_landing_loc_ref = rf_start;
	bool use_prediction = false;

	printf("t, state, com_x_r, com_y_r, com_z_r, qx_r, qx_y, qx_z, qx_w, fz0, fz1, fz2, fz3, land_x, land_y, land_z, side, use_land_prediction\n");
	for(int i = 0; i < (n_total+1); i++){
		t_query = i*dt;
		state = walking_reference_module.getState(t_query);
		walking_reference_module.getMPCRefComAndOri(t_query, x_com_ref, x_ori_ref);

		for(int j = 0; j < index_to_side.size(); j++){
			max_z_force[j] = walking_reference_module.getMaxNormalForce(j, t_query);			
		}

		// check whether to use the predicted footstep or the current foot location
		use_prediction = walking_reference_module.getFutureMPCFootstep(t_query, mpc_landing_loc_ref);

		printf("%0.3f, %i, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %i, %i\n",
				t_query, state, x_com_ref[0], x_com_ref[1], x_com_ref[2],
				x_ori_ref.x(), x_ori_ref.y(), x_ori_ref.z(), x_ori_ref.w(),
				max_z_force[0],max_z_force[1],max_z_force[2],max_z_force[3],
				mpc_landing_loc_ref.position[0], mpc_landing_loc_ref.position[1], mpc_landing_loc_ref.position[2], mpc_landing_loc_ref.robot_side, use_prediction);
	}
}

TEST(ReferenceTest, set_then_clear_footsteps){
	std::vector<int> index_to_side = {DRACO_RIGHT_FOOTSTEP, DRACO_RIGHT_FOOTSTEP,
									  DRACO_LEFT_FOOTSTEP, DRACO_LEFT_FOOTSTEP};


	WalkingReferenceTrajectoryModule walking_reference_module(index_to_side);

	// Initialize Necessary States
	// CoM Height
	Eigen::Vector3d x_com_pos_in; x_com_pos_in.setZero();
	x_com_pos_in[2] = 0.7;
	// Body Ori
	Eigen::Quaterniond x_ori_start_in; x_ori_start_in.setIdentity();
	// Footstep locations
	DracoFootstep lf_start, rf_start;
    double nominal_width = 0.333657;  // 33.3cm distance between left and right feet
	lf_start.setPosOriSide(Eigen::Vector3d(0.0, nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_LEFT_FOOTSTEP);
	rf_start.setPosOriSide(Eigen::Vector3d(0.0, -nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_RIGHT_FOOTSTEP);



	// Initialize Footsteps
	Eigen::Vector3d foot_translate(0.25, 0.1, 0.0);
	Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(-M_PI/12.0, Eigen::Vector3d::UnitZ()) );
	myUtils::pretty_print(foot_rotate, std::cout, "quat_foot_rotate");

	// stance foot is the left foot intially. So let us take a right footstep forward.
	DracoFootstep right_footstep1;
	right_footstep1.setPosOriSide(rf_start.position + foot_translate, foot_rotate*rf_start.orientation, DRACO_RIGHT_FOOTSTEP);

	DracoFootstep left_footstep1;
	left_footstep1.setPosOriSide(lf_start.position + foot_translate, lf_start.orientation, DRACO_LEFT_FOOTSTEP);

	std::cout << "rf step 1" << std::endl;
	right_footstep1.printInfo();
	std::cout << "lf step 1" << std::endl;
	left_footstep1.printInfo();


	// left then right footstep
	std::vector<DracoFootstep> footstep_list;

	double t_walk_start = 0.5;
 	walking_reference_module.setStartingConfiguration(x_com_pos_in,
													  x_ori_start_in,
								  					  lf_start,
								  					  rf_start);
	walking_reference_module.setFootsteps(t_walk_start, footstep_list);

	// clear the footsteps
	footstep_list.clear();
	walking_reference_module.setFootsteps(t_walk_start, footstep_list);



	double t_query = 0.0;
	double t_total = 2.5;
	double dt = 0.01;
	int n_total = static_cast<int>(t_total/dt);

	int state;
	Eigen::Vector3d x_com_ref;
	Eigen::Quaterniond x_ori_ref;
	Eigen::VectorXd max_z_force(index_to_side.size());

	DracoFootstep mpc_landing_loc_ref = rf_start;
	bool use_prediction = false;

	printf("t, state, com_x_r, com_y_r, com_z_r, qx_r, qx_y, qx_z, qx_w, fz0, fz1, fz2, fz3, land_x, land_y, land_z, side, use_land_prediction\n");
	for(int i = 0; i < (n_total+1); i++){
		t_query = i*dt;
		state = walking_reference_module.getState(t_query);
		walking_reference_module.getMPCRefComAndOri(t_query, x_com_ref, x_ori_ref);

		for(int j = 0; j < index_to_side.size(); j++){
			max_z_force[j] = walking_reference_module.getMaxNormalForce(j, t_query);			
		}

		// check whether to use the predicted footstep or the current foot location
		use_prediction = walking_reference_module.getFutureMPCFootstep(t_query, mpc_landing_loc_ref);

		printf("%0.3f, %i, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %i, %i\n",
				t_query, state, x_com_ref[0], x_com_ref[1], x_com_ref[2],
				x_ori_ref.x(), x_ori_ref.y(), x_ori_ref.z(), x_ori_ref.w(),
				max_z_force[0],max_z_force[1],max_z_force[2],max_z_force[3],
				mpc_landing_loc_ref.position[0], mpc_landing_loc_ref.position[1], mpc_landing_loc_ref.position[2], mpc_landing_loc_ref.robot_side, use_prediction);
	}
}


TEST(ReferenceTest, two_steps_early_contact){
	std::vector<int> index_to_side = {DRACO_RIGHT_FOOTSTEP, DRACO_RIGHT_FOOTSTEP,
									  DRACO_LEFT_FOOTSTEP, DRACO_LEFT_FOOTSTEP};


	WalkingReferenceTrajectoryModule walking_reference_module(index_to_side);

	// Initialize Necessary States
	// CoM Height
	Eigen::Vector3d x_com_pos_in; x_com_pos_in.setZero();
	x_com_pos_in[2] = 0.7;
	// Body Ori
	Eigen::Quaterniond x_ori_start_in; x_ori_start_in.setIdentity();
	// Footstep locations
	DracoFootstep lf_start, rf_start;
    double nominal_width = 0.333657;  // 33.3cm distance between left and right feet
	lf_start.setPosOriSide(Eigen::Vector3d(0.0, nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_LEFT_FOOTSTEP);
	rf_start.setPosOriSide(Eigen::Vector3d(0.0, -nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_RIGHT_FOOTSTEP);



	// Initialize Footsteps
	Eigen::Vector3d foot_translate(0.25, 0.1, 0.0);
	Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(-M_PI/12.0, Eigen::Vector3d::UnitZ()) );
	myUtils::pretty_print(foot_rotate, std::cout, "quat_foot_rotate");

	// stance foot is the left foot intially. So let us take a right footstep forward.
	DracoFootstep right_footstep1;
	right_footstep1.setPosOriSide(rf_start.position + foot_translate, foot_rotate*rf_start.orientation, DRACO_RIGHT_FOOTSTEP);

	DracoFootstep left_footstep1;
	left_footstep1.setPosOriSide(lf_start.position + foot_translate, lf_start.orientation, DRACO_LEFT_FOOTSTEP);

	std::cout << "rf step 1" << std::endl;
	right_footstep1.printInfo();
	std::cout << "lf step 1" << std::endl;
	left_footstep1.printInfo();


	// left then right footstep
	std::vector<DracoFootstep> footstep_list;
	footstep_list.push_back(right_footstep1);
	footstep_list.push_back(left_footstep1);

	double t_walk_start = 0.5;
 	walking_reference_module.setStartingConfiguration(x_com_pos_in,
													  x_ori_start_in,
								  					  lf_start,
								  					  rf_start);
	walking_reference_module.setFootsteps(t_walk_start, footstep_list);


	walking_reference_module.walking_rfs_ptr->testFunction();


	double t_query = 0.0;
	double t_total = 2.5;
	double dt = 0.01;
	int n_total = static_cast<int>(t_total/dt);

	int state;
	Eigen::Vector3d x_com_ref;
	Eigen::Quaterniond x_ori_ref;

	Eigen::VectorXd max_z_force(index_to_side.size());


	double t_early = t_walk_start + footstep_list[0].double_contact_time 
								  + footstep_list[0].contact_transition_time 
								  + footstep_list[0].swing_time/2.0;
	walking_reference_module.setEarlyFootSideContact(footstep_list[0].robot_side,  t_early);
	std::cout << "t_early = " << t_early << std::endl;

	DracoFootstep mpc_landing_loc_ref = rf_start;
	bool use_prediction = false;

	printf("t, state, com_x_r, com_y_r, com_z_r, qx_r, qx_y, qx_z, qx_w, fz0, fz1, fz2, fz3, land_x, land_y, land_z, side, use_land_prediction\n");
	for(int i = 0; i < (n_total+1); i++){
		t_query = i*dt;
		state = walking_reference_module.getState(t_query);
		walking_reference_module.getMPCRefComAndOri(t_query, x_com_ref, x_ori_ref);

		for(int j = 0; j < index_to_side.size(); j++){
			max_z_force[j] = walking_reference_module.getMaxNormalForce(j, t_query);			
		}

		// check whether to use the predicted footstep or the current foot location
		use_prediction = walking_reference_module.getFutureMPCFootstep(t_query, mpc_landing_loc_ref);

		printf("%0.3f, %i, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %i, %i\n",
				t_query, state, x_com_ref[0], x_com_ref[1], x_com_ref[2],
				x_ori_ref.x(), x_ori_ref.y(), x_ori_ref.z(), x_ori_ref.w(),
				max_z_force[0],max_z_force[1],max_z_force[2],max_z_force[3],
				mpc_landing_loc_ref.position[0], mpc_landing_loc_ref.position[1], mpc_landing_loc_ref.position[2], mpc_landing_loc_ref.robot_side, use_prediction);
	}

	// Eigen::Quaterniond qyaw( Eigen::AngleAxisd(-M_PI/3.0, Eigen::Vector3d::UnitZ()) );
	// double theta = -M_PI/4.0;
	// Eigen::Quaterniond qyaw( cos(theta/2.0), 0.0, 0.0, sin(theta/2.0)  );
	// Eigen::Quaterniond qpitch( Eigen::AngleAxisd(M_PI/12.0, Eigen::Vector3d::UnitY()) );
	// Eigen::Quaterniond qroll( Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitX()) );
	// Eigen::Quaterniond qres = qyaw*qpitch*qroll;

	Eigen::Vector3d euler_zyx = myUtils::QuatToEulerZYX(x_ori_ref);
	myUtils::pretty_print(x_ori_ref, std::cout, "x_ori_ref");
	myUtils::pretty_print(euler_zyx, std::cout, "euler_zyx");


}














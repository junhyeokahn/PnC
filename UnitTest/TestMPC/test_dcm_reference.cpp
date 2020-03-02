#include "gtest/gtest.h"
#include <PnC/DracoPnC/PredictionModule/DCMWalkingReference.hpp>
#include <PnC/DracoPnC/PredictionModule/DCMWalkingReferenceTrajectoryModule.hpp>

#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

TEST(DCMReferenceTest, footsteps){
	DCMWalkingReference dcm_reference;

	// Initialize Necessary States
	// CoM Height
	Eigen::Vector3d x_com_pos_in; x_com_pos_in.setZero();
	x_com_pos_in[2] = 0.75;
	// Body Ori
	Eigen::Quaterniond x_ori_start_in; x_ori_start_in.setIdentity();
	// Footstep locations
	DracoFootstep lf_start, rf_start;
    double nominal_width = 0.333657;  // 33.3cm distance between left and right feet
	lf_start.setPosOriSide(Eigen::Vector3d(0.0, nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_LEFT_FOOTSTEP);
	rf_start.setPosOriSide(Eigen::Vector3d(0.0, -nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_RIGHT_FOOTSTEP);

	// Initialize Footsteps
	Eigen::Vector3d foot_translate(0.25, 0.0, 0.0);
	Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );
	myUtils::pretty_print(foot_rotate, std::cout, "quat_foot_rotate");

	// stance foot is the left foot intially. So let us take a right footstep forward.
	DracoFootstep right_footstep1;
	right_footstep1.setPosOriSide(rf_start.position + foot_translate, foot_rotate*rf_start.orientation, DRACO_RIGHT_FOOTSTEP);

	DracoFootstep left_footstep1;
	left_footstep1.setPosOriSide(lf_start.position + 2.0*foot_translate, lf_start.orientation, DRACO_LEFT_FOOTSTEP);

	// stance foot is the left foot intially. So let us take a right footstep forward.
	DracoFootstep right_footstep2;
	right_footstep2.setPosOriSide(right_footstep1.position + 2.0*foot_translate, right_footstep1.orientation, DRACO_RIGHT_FOOTSTEP);

	std::cout << "rf step 1" << std::endl;
	right_footstep1.printInfo();
	std::cout << "lf step 1" << std::endl;
	left_footstep1.printInfo();
	std::cout << "rf step 2" << std::endl;
	right_footstep2.printInfo();


	// left then right footstep
	std::vector<DracoFootstep> footstep_list;
	footstep_list.push_back(right_footstep1);
	footstep_list.push_back(left_footstep1);
	footstep_list.push_back(right_footstep2);

	dcm_reference.initialize_footsteps_rvrp(footstep_list, lf_start, rf_start, x_com_pos_in);

	// print out
	dcm_reference.printBoundaryConditions();

	// set initial global start time
	double t_start = 0.25;
	dcm_reference.setInitialTime(t_start);

	// Get references
	Eigen::Vector3d dcm_ref, dcm_vel_ref, com_pos_ref, com_vel_ref, r_vrp_ref;
	dcm_ref.setZero(), dcm_vel_ref.setZero(), com_pos_ref.setZero(), com_vel_ref.setZero();
	double t = 0.0;
	double t_total = 4.0;
	double dt = 0.01;

	int N_size = int(t_total/dt);


	printf("t, dcm_x, dcm_y, dcm_z, dcm_vx, dcm_vy, dcm_vz, com_x, com_y, com_z, com_vx, com_vy, com_vz, vrp_x, vrp_y, vrp_z\n");
	for(int i = 0; i < (N_size + 1); i++){
		t = i*dt;		
		dcm_reference.get_ref_dcm(t, dcm_ref);
		dcm_reference.get_ref_dcm_vel(t, dcm_vel_ref);
		dcm_reference.get_ref_com(t, com_pos_ref);
		dcm_reference.get_ref_com_vel(t, com_vel_ref);
	  	dcm_reference.get_ref_r_vrp(t, r_vrp_ref);		
		printf("%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f \n",
			   t, dcm_ref[0], dcm_ref[1], dcm_ref[2], dcm_vel_ref[0], dcm_vel_ref[1], dcm_vel_ref[2],
			   	  com_pos_ref[0], com_pos_ref[1], com_pos_ref[2], com_vel_ref[0], com_vel_ref[1], com_vel_ref[2],
			   	  r_vrp_ref[0], r_vrp_ref[1], r_vrp_ref[2]);
	}
}

TEST(DCMTrajectoryModule, trajectory_module){
	std::vector<int> index_to_side = {DRACO_RIGHT_FOOTSTEP, DRACO_RIGHT_FOOTSTEP,
									  DRACO_LEFT_FOOTSTEP, DRACO_LEFT_FOOTSTEP};


	DCMWalkingReferenceTrajectoryModule dcm_walking_reference_module(index_to_side);

	// Initialize Necessary States
	// CoM Height
	Eigen::Vector3d x_com_pos_in; x_com_pos_in.setZero();
	x_com_pos_in[2] = 0.75;
	// Body Ori
	Eigen::Quaterniond x_ori_start_in; x_ori_start_in.setIdentity();
	// Footstep locations
	DracoFootstep lf_start, rf_start;
    double nominal_width = 0.333657;  // 33.3cm distance between left and right feet
	lf_start.setPosOriSide(Eigen::Vector3d(0.0, nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_LEFT_FOOTSTEP);
	rf_start.setPosOriSide(Eigen::Vector3d(0.0, -nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), DRACO_RIGHT_FOOTSTEP);

	// Initialize Footsteps
	Eigen::Vector3d foot_translate(0.25, 0.0, 0.0);
	Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(-M_PI/12.0, Eigen::Vector3d::UnitZ()) );
	myUtils::pretty_print(foot_rotate, std::cout, "quat_foot_rotate");

	// stance foot is the left foot intially. So let us take a right footstep forward.
	DracoFootstep right_footstep1;
	right_footstep1.setPosOriSide(rf_start.position + foot_translate, foot_rotate*rf_start.orientation, DRACO_RIGHT_FOOTSTEP);

	DracoFootstep left_footstep1;
	left_footstep1.setPosOriSide(lf_start.position + 2.0*foot_translate, lf_start.orientation, DRACO_LEFT_FOOTSTEP);

	// stance foot is the left foot intially. So let us take a right footstep forward.
	DracoFootstep right_footstep2;
	right_footstep2.setPosOriSide(right_footstep1.position + 2.0*foot_translate, rf_start.orientation, DRACO_RIGHT_FOOTSTEP);

	std::cout << "rf step 1" << std::endl;
	right_footstep1.printInfo();
	std::cout << "lf step 1" << std::endl;
	left_footstep1.printInfo();
	std::cout << "rf step 2" << std::endl;
	right_footstep2.printInfo();


	// left then right footstep
	std::vector<DracoFootstep> footstep_list;
	footstep_list.push_back(right_footstep1);
	footstep_list.push_back(left_footstep1);
	footstep_list.push_back(right_footstep2);


	double t_walk_start = 0.0;
 	dcm_walking_reference_module.setStartingConfiguration(x_com_pos_in,
													  x_ori_start_in,
								  					  lf_start,
								  					  rf_start);
	dcm_walking_reference_module.setFootsteps(t_walk_start, footstep_list);


	// Get references
	Eigen::Vector3d dcm_ref, dcm_vel_ref, com_pos_ref, com_vel_ref, r_vrp_ref;
	dcm_ref.setZero(), dcm_vel_ref.setZero(), com_pos_ref.setZero(), com_vel_ref.setZero();

	Eigen::VectorXd max_z_force(index_to_side.size());

	// Orientation references
	Eigen::Quaterniond x_ori_ref; x_ori_ref.setIdentity();
	Eigen::Vector3d ang_vel_ref, ang_acc_ref;
	ang_vel_ref.setZero(); ang_acc_ref.setZero();

	double t = 0.0;
	double t_total = 4.0;
	double dt = 0.01;

	int N_size = int(t_total/dt);
	int footstep_index = 0;
	printf("t, dcm_x, dcm_y, dcm_z, dcm_vx, dcm_vy, dcm_vz, com_x, com_y, com_z, com_vx, com_vy, com_vz, vrp_x, vrp_y, vrp_z, footstep_index, fz0, fz1, fz2, fz3, qx, qy, qz, qw, wx, wy, wz, ax, ay, az \n");
	for(int i = 0; i < (N_size + 1); i++){
		t = i*dt;		

		footstep_index = 0;
		if (dcm_walking_reference_module.whichFootstepIndexInSwing(t, footstep_index)){
			footstep_index += 1;
		}

		for(int j = 0; j < index_to_side.size(); j++){
			max_z_force[j] = dcm_walking_reference_module.getMaxNormalForce(j, t);			
		}

		dcm_walking_reference_module.dcm_reference.get_ref_dcm(t, dcm_ref);
		dcm_walking_reference_module.dcm_reference.get_ref_dcm_vel(t, dcm_vel_ref);
		// dcm_walking_reference_module.dcm_reference.get_ref_com(t, com_pos_ref);
		// dcm_walking_reference_module.dcm_reference.get_ref_com_vel(t, com_vel_ref);
	  	dcm_walking_reference_module.dcm_reference.get_ref_r_vrp(t, r_vrp_ref);		
		// dcm_walking_reference_module.dcm_reference.get_ref_ori_ang_vel_acc(t, x_ori_ref, ang_vel_ref, ang_acc_ref);

		dcm_walking_reference_module.getMPCRefQuatAngVelAngAcc(t, x_ori_ref, ang_vel_ref, ang_acc_ref);
		dcm_walking_reference_module.getMPCRefComPosandVel(t, com_pos_ref, com_vel_ref);

		printf("%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %i, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f \n",
			   t, dcm_ref[0], dcm_ref[1], dcm_ref[2], dcm_vel_ref[0], dcm_vel_ref[1], dcm_vel_ref[2],
			   	  com_pos_ref[0], com_pos_ref[1], com_pos_ref[2], com_vel_ref[0], com_vel_ref[1], com_vel_ref[2],
			   	  r_vrp_ref[0], r_vrp_ref[1], r_vrp_ref[2], footstep_index,
			   	  max_z_force[0], max_z_force[1], max_z_force[2], max_z_force[3],
			   	  x_ori_ref.x(), x_ori_ref.y(), x_ori_ref.z(), x_ori_ref.w(), ang_vel_ref[0], ang_vel_ref[1], ang_vel_ref[2], ang_acc_ref[0], ang_acc_ref[1], ang_acc_ref[2]);
	}


	// Eigen::Quaterniond start( Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) );
	// Eigen::Quaterniond rz( Eigen::AngleAxisd(M_PI/12.0, Eigen::Vector3d::UnitZ()) );
	// HermiteQuaternionCurve curve(start, Eigen::Vector3d(0.0,0.0,0.0),
	// 							 rz, Eigen::Vector3d(0,0.0,2));
	// int N_test = int(1.0/dt);
	// double s = 0;
	// printf("s, wx, wy, wz\n");
	// for(int i = 0; i < (N_test + 1); i++){
	// 	s = i*dt;
	// 	curve.getAngularVelocity(s, ang_vel_ref);
	// 	printf("%0.3f, %0.3f, %0.3f, %0.3f \n", s, ang_vel_ref[0], ang_vel_ref[1], ang_vel_ref[2]);
	// }


}
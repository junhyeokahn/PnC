#include "gtest/gtest.h"
#include <PnC/DracoPnC/PredictionModule/DCMWalkingReference.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

TEST(DCMReferenceTest, footsteps){
	DCMWalkingReference dcm_reference;

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



}
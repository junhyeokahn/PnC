#include "gtest/gtest.h"
#include <PnC/PlannerSet/DCMPlanner/Footstep.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>


TEST(DCMPlanner, footstep_object){
	// Footstep locations
	Footstep lf_start, rf_start;
    double nominal_width = 0.333657;  // 33.3cm distance between left and right feet
	lf_start.setPosOriSide(Eigen::Vector3d(0.0, nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), LEFT_FOOTSTEP);
	rf_start.setPosOriSide(Eigen::Vector3d(0.0, -nominal_width, 0.0), Eigen::Quaterniond(1, 0, 0, 0), RIGHT_FOOTSTEP);

	// Initialize Footsteps
	Eigen::Vector3d foot_translate(0.25, 0.1, 0.0);
	Eigen::Quaterniond foot_rotate( Eigen::AngleAxisd(-M_PI/12.0, Eigen::Vector3d::UnitZ()) );
	myUtils::pretty_print(foot_rotate, std::cout, "quat_foot_rotate");

	// Set Footsteps
	Footstep right_footstep1;
	right_footstep1.setPosOriSide(rf_start.position + foot_translate, foot_rotate*rf_start.orientation, RIGHT_FOOTSTEP);

	Footstep left_footstep1;
	left_footstep1.setPosOriSide(lf_start.position + foot_translate, lf_start.orientation, LEFT_FOOTSTEP);

	std::cout << "RF step 1" << std::endl;
	right_footstep1.printInfo();
	std::cout << "LF step 1" << std::endl;
	left_footstep1.printInfo();
}
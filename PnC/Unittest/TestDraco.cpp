#include "gtest/gtest.h"

#include <drake/common/find_resource.h>
#include <drake/multibody/ik_options.h>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_constraint.h>
#include <drake/multibody/rigid_body_ik.h>
#include <drake/multibody/rigid_body_tree.h>

#include "eigen_matrix_compare.h"
#include "Configuration.h"
#include "RobotSystem.hpp"

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

TEST(TestDraco, CoM) {

    /////////////////////
    // load draco to dart
    /////////////////////
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr dart_model = urdfLoader.parseSkeleton(
            THIS_COM"RobotSystem/RobotModel/Robot/Draco/DracoNoVisualDart.urdf");
    std::cout << "com at initial q" << std::endl;
    std::cout << dart_model->getCOM() << std::endl;
    std::cout << "right ankle at initial q" << std::endl;
    std::cout << dart_model->getBodyNode("rAnkle")->getWorldTransform().translation() << std::endl;
    std::cout << "total mass" << std::endl;
    std::cout << dart_model->getMass() << std::endl;
}

#include <towr_plus/locomotion_task.h>
#include <towr_plus/terrain/examples/height_map_examples.h>

LocomotionTask::LocomotionTask(const std::string &name) {
  name_ = name;
  int num_leg(2);
  initial_ee_motion_lin.resize(num_leg);
  initial_ee_motion_ang.resize(num_leg);
}

LocomotionTask::~LocomotionTask() {}

void LocomotionTask::from_yaml(const YAML::Node &node) {
  std::string terrain_type;
  Eigen::VectorXd tmp_vec;
  try {
    ReadParameter(node, "initial_base_lin", initial_base_lin);
    ReadParameter(node, "initial_base_ang", initial_base_ang);
    for (auto ee : {L, R}) {
      ReadParameter(node["initial_ee_motion_lin"], std::to_string(ee), tmp_vec);
      for (auto dim : {X, Y, Z})
        initial_ee_motion_lin.at(ee)(dim) = tmp_vec(dim);
      ReadParameter(node["initial_ee_motion_ang"], std::to_string(ee), tmp_vec);
      for (auto dim : {X, Y, Z})
        initial_ee_motion_ang.at(ee)(dim) = tmp_vec(dim);
    }
    ReadParameter(node, "final_base_lin", final_base_lin);
    ReadParameter(node, "final_base_ang", final_base_ang);

    ReadParameter(node, "terrain_type", terrain_type);

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }

  if (terrain_type == "flat_ground") {
    terrain = std::make_shared<FlatGround>(0.);
  } else if (terrain_type == "block") {
    terrain = std::make_shared<Block>();
  } else if (terrain_type == "stair") {
    terrain = std::make_shared<Stairs>();
  } else if (terrain_type == "slope") {
    terrain = std::make_shared<Slope>();
  } else if (terrain_type == "chimney") {
    terrain = std::make_shared<Chimney>();
  } else if (terrain_type == "lr_chimney") {
    terrain = std::make_shared<ChimneyLR>();
  } else {
    std::cout << "Wrong Terrain Type" << std::endl;
    exit(0);
  }
}

void LocomotionTask::print_info() {
  std::cout << "Locomotion Task for " << name_ << std::endl;
}

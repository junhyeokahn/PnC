#include <PnC/DracoPnC/PredictionModule/draco_footstep.hpp>

DracoFootstep::DracoFootstep(){
	position.setZero();
	orientation.setIdentity();
  robot_side = DRACO_LEFT_FOOTSTEP;
  common_initialization();  
}

DracoFootstep::DracoFootstep(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in){
  position = pos_in;
  orientation = quat_in;
  robot_side = robot_side_in;
  common_initialization();  
}

DracoFootstep::~DracoFootstep(){	
}

void DracoFootstep::setPosOriSide(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in){
  position = pos_in;
  orientation = quat_in;  
  robot_side = robot_side_in;
  R_ori = orientation.toRotationMatrix(); 
  updateContactLocations();
}

void DracoFootstep::setPosOri(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in){
  position = pos_in;
  orientation = quat_in;
  R_ori = orientation.toRotationMatrix(); 
  updateContactLocations();
}

void DracoFootstep::setRightSide(){  
  robot_side = DRACO_RIGHT_FOOTSTEP;
}
void DracoFootstep::setLeftSide(){  
  robot_side = DRACO_LEFT_FOOTSTEP;
}

void DracoFootstep::setMidFoot(){  
  robot_side = DRACO_MID_FOOTSTEP;
}

void DracoFootstep::common_initialization(){
	R_ori = orientation.toRotationMatrix();	

  // Initialize contact locations list
  local_contact_point_list.reserve(2);
  global_contact_point_list.reserve(2);
  for(int i = 0; i < 2; i++){
    local_contact_point_list.push_back(Eigen::Vector3d::Zero(3));
    global_contact_point_list.push_back(Eigen::Vector3d::Zero(3));
  }

  // Set Local Frame Contact Point List
  local_contact_point_list[0] = Eigen::Vector3d(sole_length/2.0, 0.0, 0.0);
  local_contact_point_list[1] = Eigen::Vector3d(-sole_length/2.0, 0.0, 0.0);
  // Set Global Frame Contact Point List
  updateContactLocations();

  // std::cout << "[Footstep] Data Object constructed" << std::endl;
  // std::cout << "[Footstep] Size of local contact point list = " << local_contact_point_list.size() << std::endl;
  // std::cout << "[Footstep] Size of global_contact_point_list = " << local_contact_point_list.size() << std::endl;


}

void DracoFootstep::updateContactLocations(){
  // Update global location of contact points based on the pose and orientation of the foot
  for(int i = 0; i < global_contact_point_list.size(); i++){
    global_contact_point_list[i] = R_ori*local_contact_point_list[i] + position;
  }
}


void DracoFootstep::printInfo(){
  if ((robot_side == DRACO_LEFT_FOOTSTEP) || (robot_side == DRACO_RIGHT_FOOTSTEP)){
    std::cout << "side = " << (robot_side == DRACO_LEFT_FOOTSTEP ? "DRACO_LEFT_FOOTSTEP" : "DRACO_RIGHT_FOOTSTEP") << std::endl;    
  }else if (robot_side == DRACO_MID_FOOTSTEP){
    std::cout << "DRACO_MID_FOOTSTEP" << std::endl;    
  }

	std::cout << "pos: " << position[0] << ", " << position[1] << ", " << position[2] << std::endl;
  std::cout << "ori: " << orientation.x() << ", " 
                     << orientation.y() << ", " 
                     << orientation.z() << ", "
                     << orientation.w() << std::endl;
}

void DracoFootstep::computeMidfeet(const DracoFootstep & footstep1, const DracoFootstep & footstep2, DracoFootstep & midfeet){
  midfeet.position = 0.5*(footstep1.position + footstep2.position);  
  midfeet.orientation = footstep1.orientation.slerp(0.5, footstep2.orientation);
  midfeet.R_ori = midfeet.orientation.toRotationMatrix(); 
  midfeet.robot_side = DRACO_MID_FOOTSTEP;
}

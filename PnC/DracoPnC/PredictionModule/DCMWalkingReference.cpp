  #include <PnC/DracoPnC/PredictionModule/DCMWalkingReference.hpp>

int const DCMWalkingReference::DCM_RL_SWING_VRP_TYPE = 1;
int const DCMWalkingReference::DCM_LL_SWING_VRP_TYPE = 2;
int const DCMWalkingReference::DCM_TRANSFER_VRP_TYPE = 3;
int const DCMWalkingReference::DCM_END_VRP_TYPE = 4;
int const DCMWalkingReference::DCM_MIDSTEP_TRANSFER_VRP_TYPE = 5;

DCMWalkingReference::DCMWalkingReference(){
    initial_leftfoot_stance.setLeftSide();
    initial_rightfoot_stance.setRightSide();
    std::cout << "[DCMWalkingReference] Constructed" << std::endl;
} 

DCMWalkingReference::~DCMWalkingReference(){

}

// Sets the desired CoM Height
void DCMWalkingReference::setCoMHeight(double z_vrp_in){
  z_vrp = z_vrp_in;
  b = std::sqrt(z_vrp/gravity); // set time constant of DCM dynamics  
} 

void DCMWalkingReference::setRobotMass(double mass){
  robot_mass = mass;
}

void DCMWalkingReference::setInitialTime(double t_start_in){
    t_start = t_start_in;
}

double DCMWalkingReference::getInitialTime(){
  return t_start;
}

void DCMWalkingReference::setInitialOri(const Eigen::Quaterniond initial_ori_in){
  initial_ori = initial_ori_in;
}


void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, 
                                                        const DracoFootstep & initial_footstance,
                                                        bool clear_list){
  // Store the input footstep list
  footstep_list = input_footstep_list;
  if (input_footstep_list.size() == 0){ 
    return;
  }
  // clear rvrp list if true
  if (clear_list){
    rvrp_list.clear();    
    rvrp_type_list.clear();
    rvrp_index_to_footstep_index.clear();
  }

  // Create an rvrp for the stance leg
  Eigen::Vector3d current_rvrp(0, 0, z_vrp); // From foot local frame
  Eigen::Vector3d current_stance_rvrp(0, 0, z_vrp); // The stance leg from the foot local frame
  Eigen::Vector3d left_stance_rvrp(0, 0, z_vrp); // a left stance leg from the foot local frame
  Eigen::Vector3d right_stance_rvrp(0, 0, z_vrp); // a right stance leg from the foot local frame

  current_stance_rvrp = initial_footstance.R_ori * current_stance_rvrp + initial_footstance.position;
  left_stance_rvrp = current_stance_rvrp;
  right_stance_rvrp = current_stance_rvrp;

  // Add an rvrp to transfer to the stance leg
  rvrp_list.push_back(current_stance_rvrp);

  int previous_step = initial_footstance.robot_side;

  for(int i = 0; i < input_footstep_list.size(); i++){
    // initialize rvrp to [0, 0, z_vrp] in terms of the foot's local frame
    current_rvrp.setZero(); current_rvrp[2] = z_vrp;
    // Get the world frame representation
    current_rvrp = input_footstep_list[i].R_ori * current_rvrp + input_footstep_list[i].position;

    // Set the correct stance rvrp
    current_stance_rvrp = input_footstep_list[i].robot_side == DRACO_LEFT_FOOTSTEP ? right_stance_rvrp : left_stance_rvrp;
    // If this is the last step, use the average rvrp between the stance and current rvrp
    if (i == (input_footstep_list.size() - 1)){
      current_rvrp = 0.5*(current_rvrp + current_stance_rvrp);
    }

    // ----------- Begin handling same robot side footsteps -------------
    // If taking a footstep on the same side, first go to the stance foot
    if (input_footstep_list[i].robot_side == previous_step){
      // Add a new rvrp
      rvrp_type_list.push_back(DCM_TRANSFER_VRP_TYPE);
      rvrp_list.push_back(current_stance_rvrp);
    }
    else{
      // otherwise, update the correct stance to the latest rvrp
      if (input_footstep_list[i].robot_side == DRACO_LEFT_FOOTSTEP){
        left_stance_rvrp = current_rvrp;
      }else{
        right_stance_rvrp = current_rvrp;
      }
    }
    // -----------------------------------------------------------------

    // Specify the right swing VRP type
    input_footstep_list[i].robot_side == DRACO_LEFT_FOOTSTEP ? rvrp_type_list.push_back(DCM_LL_SWING_VRP_TYPE) : rvrp_type_list.push_back(DCM_RL_SWING_VRP_TYPE);
    // Mark the current RVRP index to correspond to this footstep swing.
    rvrp_index_to_footstep_index[ rvrp_list.size() - 1 ] = i; 

    // Land in the midfeet every step.
    // if (i != input_footstep_list.size()){
    //   rvrp_list.push_back(0.5*(current_rvrp + current_stance_rvrp));
    //   rvrp_type_list.push_back(DCM_MIDSTEP_TRANSFER_VRP_TYPE);      
    //   rvrp_list.push_back(current_rvrp);
    // }else{
    //     rvrp_list.push_back(current_rvrp);
    // }

    // // If not the last step, after every 2nd step, add a transfer RVRP
    // if ((i != input_footstep_list.size() - 1) && ((i+1) % 2 == 0)){
    //   rvrp_list.push_back(0.5*(current_rvrp + current_stance_rvrp));
    //   rvrp_type_list.push_back(DCM_MIDSTEP_TRANSFER_VRP_TYPE);      
    //   rvrp_list.push_back(current_rvrp);
    // }else{
    //     rvrp_list.push_back(current_rvrp);
    // }

    // Add this rvrp to the list and also populate the DCM states
    rvrp_list.push_back(current_rvrp);

    // The following is a hack to ensure convergence every midstep.
    // Please remove the if statement
    // If this is not the last step, add a midstep transfer VRP
    // if (i != (input_footstep_list.size() - 1)){
      // rvrp_type_list.push_back(DCM_MIDSTEP_TRANSFER_VRP_TYPE);
      // rvrp_list.push_back(0.5*(current_rvrp + current_stance_rvrp));
      // rvrp_type_list.push_back(DCM_MIDSTEP_TRANSFER_VRP_TYPE);
      // rvrp_list.push_back(current_rvrp);
    // }


    // Update previous_step side 
    previous_step = input_footstep_list[i].robot_side;
  }
  // Add final RVRP as ending
  rvrp_type_list.push_back(DCM_END_VRP_TYPE);

  // Compute DCM states
  computeDCM_states();
}

void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & initial_footstance, const Eigen::Vector3d & initial_rvrp){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  rvrp_list.clear();
  rvrp_type_list.clear();
  rvrp_index_to_footstep_index.clear();
  dcm_ini_list.clear();
  dcm_eos_list.clear(); 
  dcm_ini_DS_list.clear();
  dcm_vel_ini_DS_list.clear();
  dcm_acc_ini_DS_list.clear();
  dcm_end_DS_list.clear();
  dcm_vel_end_DS_list.clear();
  dcm_acc_end_DS_list.clear();

  // Add the initial virtual repellant point. 
  rvrp_list.push_back(initial_rvrp); 
  // Specify that this is the eos for the previous rvrp
  rvrp_type_list.push_back(DCM_TRANSFER_VRP_TYPE);

  // Add the remaining virtual repellant points   
  initialize_footsteps_rvrp(input_footstep_list, initial_footstance);
}

void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & left_footstance, const DracoFootstep & right_footstance){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  Eigen::Vector3d initial_rvrp; 
  get_average_rvrp(left_footstance, right_footstance, initial_rvrp);
  // Set the stance leg
  if (input_footstep_list[0].robot_side == DRACO_LEFT_FOOTSTEP){
    initialize_footsteps_rvrp(input_footstep_list, right_footstance, initial_rvrp);
  }else{
    initialize_footsteps_rvrp(input_footstep_list, left_footstance, initial_rvrp);
  }

}

void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & left_footstance, const DracoFootstep & right_footstance, const Eigen::Vector3d & initial_com){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  // Set the stance leg
  if (input_footstep_list[0].robot_side == DRACO_LEFT_FOOTSTEP){
    initialize_footsteps_rvrp(input_footstep_list, right_footstance, initial_com);
  }else{
    initialize_footsteps_rvrp(input_footstep_list, left_footstance, initial_com);
  }

}

double DCMWalkingReference::get_t_step(const int & step_i){
  // Use transfer time for double support and overall step time for swing types
  if (rvrp_type_list[step_i] == DCMWalkingReference::DCM_TRANSFER_VRP_TYPE){
    return t_transfer + t_ds; 
  }else if ((rvrp_type_list[step_i] == DCMWalkingReference::DCM_RL_SWING_VRP_TYPE) || 
            (rvrp_type_list[step_i] == DCMWalkingReference::DCM_LL_SWING_VRP_TYPE)){
    return t_ss + t_ds; // every swing has a double support transfer
  }else if (rvrp_type_list[step_i] == DCMWalkingReference::DCM_END_VRP_TYPE){
    return t_ds*(1-alpha_ds);
  }else if (rvrp_type_list[step_i] == DCM_MIDSTEP_TRANSFER_VRP_TYPE){
    return t_midstep_transfer;
  }
}

void DCMWalkingReference::get_average_rvrp(const DracoFootstep & footstance_1, const DracoFootstep & footstance_2, Eigen::Vector3d & average_rvrp){
  Eigen::Vector3d desired_rvrp(0, 0, z_vrp); // From foot local frame
  average_rvrp = 0.5*((footstance_1.R_ori*desired_rvrp + footstance_1.position) + (footstance_2.R_ori*desired_rvrp + footstance_2.position));
}

Eigen::Vector3d DCMWalkingReference::computeDCM_ini_i(const Eigen::Vector3d & r_vrp_d_i, const double & t_step, const Eigen::Vector3d & dcm_eos_i){
  return r_vrp_d_i + std::exp(-t_step/b)*(dcm_eos_i - r_vrp_d_i);
}



void DCMWalkingReference::computeDCM_states(){
  // Clear the dcm lists
  dcm_ini_list.clear();
  dcm_eos_list.clear();
  dcm_ini_DS_list.clear(); dcm_vel_ini_DS_list.clear(); dcm_acc_ini_DS_list.clear();
  dcm_end_DS_list.clear(); dcm_vel_end_DS_list.clear(); dcm_acc_end_DS_list.clear(); 
  dcm_P.clear();
  dcm_minjerk.clear();

  // Resize DCM lists to be equal to the size of the rvrp  list
  dcm_ini_list.resize(rvrp_list.size());
  dcm_eos_list.resize(rvrp_list.size());
  dcm_ini_DS_list.resize(rvrp_list.size()); dcm_vel_ini_DS_list.resize(rvrp_list.size()); dcm_acc_ini_DS_list.resize(rvrp_list.size()); 
  dcm_end_DS_list.resize(rvrp_list.size()); dcm_vel_end_DS_list.resize(rvrp_list.size()); dcm_acc_end_DS_list.resize(rvrp_list.size());
  dcm_P.resize(rvrp_list.size()); 
  dcm_minjerk.resize(rvrp_list.size());

  // Use backwards recursion to compute the initial and final dcm states
  double t_step = 0.0;
  // Last element of the DCM end of step list is equal to the last rvrp.
  dcm_eos_list.back() = rvrp_list.back();
  for (int i = dcm_ini_list.size()-1; i >= 0; i--){
    // Get the t_step to use for backwards integration
    t_step = get_t_step(i);
    // Compute dcm_ini for step i
    dcm_ini_list[i] = computeDCM_ini_i(rvrp_list[i], t_step, dcm_eos_list[i]);

    // Set dcm_eos for step i-1
    if (i > 0){
      dcm_eos_list[i-1] = dcm_ini_list[i];
    }
  }

  // Find boundary conditions for the Polynomial interpolator
  for (int i = 0; i < rvrp_list.size(); i++){
   // compute boundary conditions
   dcm_ini_DS_list[i] = computeDCM_iniDS_i(i, alpha_ds*t_ds);
   dcm_vel_ini_DS_list[i] = computeDCMvel_iniDS_i(i, alpha_ds*t_ds);
   dcm_acc_ini_DS_list[i] = computeDCMacc_iniDS_i(i, alpha_ds*t_ds);
   dcm_end_DS_list[i] = computeDCM_eoDS_i(i, (1.0-alpha_ds)*t_ds);
   dcm_vel_end_DS_list[i] = computeDCMvel_eoDS_i(i, (1.0-alpha_ds)*t_ds);
   dcm_acc_end_DS_list[i] = computeDCMacc_eoDS_i(i, (1.0-alpha_ds)*t_ds);          
  }

  // for (int i = 0; i < rvrp_list.size(); i++){
  //   if (i == 0){
  //     dcm_end_DS_list[i] = computeDCM_eoDS_i(i, t_transfer + (1.0-alpha_ds)*t_ds);
  //     dcm_vel_end_DS_list[i] = computeDCMvel_eoDS_i(i, t_transfer + (1.0-alpha_ds)*t_ds);
  //     dcm_acc_end_DS_list[i] = computeDCMacc_eoDS_i(i, t_transfer + (1.0-alpha_ds)*t_ds);          
  //   }else{
  //     dcm_end_DS_list[i] = computeDCM_eoDS_i(i, (1.0-alpha_ds)*t_ds);
  //     dcm_vel_end_DS_list[i] = computeDCMvel_eoDS_i(i, (1.0-alpha_ds)*t_ds);
  //     dcm_acc_end_DS_list[i] = computeDCMacc_eoDS_i(i, (1.0-alpha_ds)*t_ds);          
  //   }

  // }

  // printBoundaryConditions();

  // compute polynomial interpolator matrix
  double Ts = t_ds; // set transfer duration time
  for (int i = 0; i < rvrp_list.size(); i++){
    Ts = get_polynomial_duration(i);
    dcm_P[i] = polynomialMatrix(Ts, dcm_ini_DS_list[i], dcm_vel_ini_DS_list[i],
                                    dcm_end_DS_list[i], dcm_vel_end_DS_list[i]);

    dcm_minjerk[i] =  MinJerkCurveVec(dcm_ini_DS_list[i], dcm_vel_ini_DS_list[i], dcm_acc_ini_DS_list[i], 
                                      dcm_end_DS_list[i], dcm_vel_end_DS_list[i], dcm_acc_end_DS_list[i],
                                      Ts);


  }

  // Compute the total trajectory time.
  compute_total_trajectory_time();
  // Compute the reference com
  compute_reference_com();
  // Compute the reference ori
  compute_reference_pelvis_ori();
}

double DCMWalkingReference::get_eoDS_transition_time(){
  return (1.0-alpha_ds)*t_ds;
}

double DCMWalkingReference::get_iniDS_transition_time(){
  return alpha_ds*t_ds;
}


void DCMWalkingReference::printBoundaryConditions(){
  Eigen::Vector3d val;
  for (int i = 0; i < rvrp_list.size(); i++){
    val = rvrp_list[i];
    std::cout << "i:" << i << " " << std::endl;
    myUtils::pretty_print(val, std::cout, "  vrp:");
    std::cout << "  type:" << rvrp_type_list[i] << std::endl;
  }

  for (int i = 0; i < dcm_ini_list.size(); i++){
    val = dcm_ini_list[i];
    std::cout << "i:" << i << " " << std::endl;
    myUtils::pretty_print(val, std::cout, "  dcm_ini:");
    val = dcm_eos_list[i];
    myUtils::pretty_print(val, std::cout, "  dcm_eos:");
  }

  for (int i = 0; i < rvrp_list.size(); i++){
    val = dcm_ini_DS_list[i];
    std::cout << "i:" << i << " " << std::endl;
    myUtils::pretty_print(val, std::cout, "  dcm_ini_DS:");
    val = dcm_end_DS_list[i];
    myUtils::pretty_print(val, std::cout, "  dcm_end_DS:");
    val = dcm_vel_ini_DS_list[i];
    myUtils::pretty_print(val, std::cout, "  dcm_vel_ini_DS:");
    val = dcm_vel_end_DS_list[i];
    myUtils::pretty_print(val, std::cout, "  dcm_vel_end_DS:");
  }  

  for(int i = 0; i < rvrp_list.size(); i++){
    printf("%i, %0.3f, %0.3f, %0.3f, %0.3f\n", i, get_t_step_start(i),
                                                get_t_step_end(i),
                                                get_double_support_t_start(i),  
                                                get_double_support_t_end(i));
  }

}

// Total trajectory time from t_start
void DCMWalkingReference::compute_total_trajectory_time(){
  t_end = 0.0;
  for (int i = 0; i < rvrp_list.size(); i++){
    t_end += get_t_step(i);
  }
  // compute settling time 
  double t_settle = -b*log(1.0 - percentage_settle);  
  t_end += t_settle;
}

double DCMWalkingReference::get_polynomial_duration(const int step_index){
  // first step has polynomial duration of only ending double support
  if (step_index == 0){
    return (1.0-alpha_ds)*t_ds; // t_transfer + 
  } 
  else if (step_index == (rvrp_list.size() - 1)){
    return t_ds; //alpha_ds*t_ds; // Not sure why... But the final duration must not be below t_ds.
  }
  return t_ds;
}

Eigen::Vector3d DCMWalkingReference::computeDCM_iniDS_i(const int & step_index, const double t_DS_ini){
  if (step_index == 0){
    // return rvrp_list.front();
    return get_DCM_exp(step_index, 0.0);
  }
  return rvrp_list[step_index - 1] + std::exp(-t_DS_ini/b) * (dcm_ini_list[step_index] - rvrp_list[step_index - 1]);
}

Eigen::Vector3d DCMWalkingReference::computeDCM_eoDS_i(const int & step_index, const double t_DS_end){
  // Set Boundary condition. Last element of eoDS is equal to the last element of the rvrp list
  if (step_index == (rvrp_list.size() - 1)){
    return rvrp_list.back();
  }
  // else if (step_index == 0){
  //   return dcm_ini_DS_list[step_index + 1];
  // }
  return rvrp_list[step_index] + std::exp(t_DS_end/b) * (dcm_ini_list[step_index] - rvrp_list[step_index]);
}

Eigen::Vector3d DCMWalkingReference::computeDCMvel_iniDS_i(const int & step_index, const double t_DS_ini){
  if (step_index == 0){
    // return Eigen::Vector3d::Zero();
    return get_DCM_vel_exp(step_index, 0.0);
  }

  return (1.0/b)*std::exp(-t_DS_ini/b) * (dcm_ini_list[step_index] - rvrp_list[step_index - 1]);
}

Eigen::Vector3d DCMWalkingReference::computeDCMvel_eoDS_i(const int & step_index, const double t_DS_end){
  // Set Boundary condition. Velocities at the very end are always 0.0
  if (step_index == (rvrp_list.size() - 1)){
    return Eigen::Vector3d::Zero();
  }
  // else if (step_index == 0){
  //   return dcm_vel_ini_DS_list[step_index + 1];
  // }   
  return (1.0/b)*std::exp(t_DS_end/b) * (dcm_ini_list[step_index] - rvrp_list[step_index]);
}


Eigen::Vector3d DCMWalkingReference::computeDCMacc_iniDS_i(const int & step_index, const double t_DS_ini){
  if (step_index == 0){
    // return Eigen::Vector3d::Zero();
    return get_DCM_acc_exp(step_index, 0.0);
  }  
  return (1.0/(std::pow(b,2)))*std::exp(-t_DS_ini/b) * (dcm_ini_list[step_index] - rvrp_list[step_index - 1]);
}

Eigen::Vector3d DCMWalkingReference::computeDCMacc_eoDS_i(const int & step_index, const double t_DS_end){
  // Set Boundary condition. Accelerations at the very end are always 0.0
  if (step_index == (rvrp_list.size() - 1)){
    return Eigen::Vector3d::Zero();
  } 
  return (1.0/(std::pow(b,2)))*std::exp(t_DS_end/b) * (dcm_ini_list[step_index] - rvrp_list[step_index]);
}


Eigen::MatrixXd DCMWalkingReference::polynomialMatrix(const double Ts,
                                                      const Eigen::Vector3d & dcm_ini, const Eigen::Vector3d & dcm_vel_ini,
                                                      const Eigen::Vector3d & dcm_end, const Eigen::Vector3d & dcm_vel_end){

  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(4,4);

  // Construct matrix mat
  mat(0,0) = 2.0/std::pow(Ts, 3);  mat(0,1) = 1.0/std::pow(Ts, 2); mat(0,2) = -2.0/std::pow(Ts, 3); mat(0,3) = 1.0/std::pow(Ts, 2);
  mat(1,0) = -3.0/std::pow(Ts, 2); mat(1,1) = -2.0/Ts;             mat(1,2) = 3.0/std::pow(Ts, 2);  mat(1,3) = -1.0/Ts;
                                   mat(2,1) = 1.0;
  mat(3,0) = 1.0;

  Eigen::MatrixXd bound = Eigen::MatrixXd::Zero(4, 3);
  bound.row(0) = dcm_ini;
  bound.row(1) = dcm_vel_ini;
  bound.row(2) = dcm_end;
  bound.row(3) = dcm_vel_end;

  return mat*bound; 
}

// Returns the DCM exponential for the requested step index
Eigen::Vector3d DCMWalkingReference::get_DCM_exp(const int & step_index, const double & t){
  // Get t_step
  double t_step = get_t_step(step_index);
  // Clamp time value
  double time = clampDOUBLE(t, 0.0, t_step);

  return rvrp_list[step_index] + std::exp( (time-t_step) / b)*(dcm_eos_list[step_index] - rvrp_list[step_index]);
}

Eigen::Vector3d DCMWalkingReference::get_DCM_vel_exp(const int & step_index, const double & t){
  // Get t_step
  double t_step = get_t_step(step_index);
  // Clamp time value
  double time = clampDOUBLE(t, 0.0, t_step);

  return (1.0/b)*std::exp( (time-t_step) / b)*(dcm_eos_list[step_index] - rvrp_list[step_index]);
}

Eigen::Vector3d DCMWalkingReference::get_DCM_acc_exp(const int & step_index, const double & t){
  // Get t_step
  double t_step = get_t_step(step_index);
  // Clamp time value
  double time = clampDOUBLE(t, 0.0, t_step);

  return (1.0/(std::pow(b,2)))*std::exp( (time-t_step) / b)*(dcm_eos_list[step_index] - rvrp_list[step_index]);  
}


// Returns the DCM double support polynomial interpolation for the requested step_index.
// time, t, is clamped between 0.0 and t_step.
Eigen::Vector3d DCMWalkingReference::get_DCM_DS_poly(const int & step_index, const double & t){
  double Ts = get_polynomial_duration(step_index);
  double time = clampDOUBLE(t, 0.0, Ts);

  Eigen::VectorXd t_mat = Eigen::VectorXd::Zero(4);
  t_mat[0] = std::pow(time, 3);  t_mat[1] = std::pow(time, 2);  t_mat[2] = time; t_mat[3] = 1.0; 
  Eigen::Vector3d result = t_mat.transpose()*dcm_P[step_index];

  return result;
}

// Returns the DCM double support velocity polynomial interpolation for the requested step_index.
// time, t, is clamped between 0.0 and t_step.
Eigen::Vector3d DCMWalkingReference::get_DCM_DS_vel_poly(const int & step_index, const double & t){
  double Ts = get_polynomial_duration(step_index);
  double time = clampDOUBLE(t, 0.0, Ts);

  Eigen::VectorXd t_mat = Eigen::VectorXd::Zero(4);
  t_mat[0] = 3.0*std::pow(time, 2);  t_mat[1] = 2.0*time;  t_mat[2] = 1.0; 
  Eigen::Vector3d result = t_mat.transpose()*dcm_P[step_index];

  return result;
}


// Returns the DCM double support min jerk interpolation for the requested step_index.
// time, t, is clamped between 0.0 and t_step.
Eigen::Vector3d DCMWalkingReference::get_DCM_DS_minjerk(const int & step_index, const double & t){
  double Ts = get_polynomial_duration(step_index);
  double time = clampDOUBLE(t, 0.0, Ts);  
  return dcm_minjerk[step_index].evaluate(time);
}

// Returns the DCM double support velocity min jerk interpolation for the requested step_index.
// time, t, is clamped between 0.0 and t_step.
Eigen::Vector3d DCMWalkingReference::get_DCM_DS_vel_minjerk(const int & step_index, const double & t){
  double Ts = get_polynomial_duration(step_index);
  double time = clampDOUBLE(t, 0.0, Ts);  
  return dcm_minjerk[step_index].evaluateFirstDerivative(time);
}


void DCMWalkingReference::get_ref_dcm(const double t, Eigen::Vector3d & dcm_out){
  // Don't process if the list of VRPs is empty
  if (rvrp_list.size() == 0){
    return;
  }

  // Reference DCM is the RVRP in the beginning.
  if (t < t_start){
    dcm_out = rvrp_list.front();
    return;
  }

  // offset time and clamp. t_start is global start time.
  double time = clampDOUBLE(t - t_start, 0.0, t_end);
  
  // Continouous Interpolation
  // interpolation index to use
  int step_index = which_step_index_to_use(time);  
  double local_time;
  if (time <= get_double_support_t_end(step_index)){
    // Use Polynomial interpolation
    local_time = time - get_double_support_t_start(step_index);
    dcm_out = get_DCM_DS_poly(step_index, local_time);
    // dcm_out = get_DCM_DS_minjerk(step_index, local_time);
  }else{ // Use exponential interpolation
    local_time = time - get_t_step_start(step_index);
    dcm_out = get_DCM_exp(step_index, local_time);
  }

  // Discountinuous
  // int step_index = which_step_index(time);
  // double local_time = time - get_t_step_start(step_index);
  // dcm_out = get_DCM_exp(step_index, local_time);

}

void DCMWalkingReference::get_ref_dcm_vel(const double t, Eigen::Vector3d & dcm_vel_out){
  // Don't process if the list of VRPs is empty
  if (rvrp_list.size() == 0){
    return;
  }
  // Velocities are zero before time starts 
  if (t < t_start){
    dcm_vel_out.setZero();
    return;
  }

  // offset time and clamp. t_start is global start time.
  double time = clampDOUBLE(t - t_start, 0.0, t_end);
  
  // Continouous Interpolation
  int step_index = which_step_index_to_use(time);  
  double local_time;

  if (time <= get_double_support_t_end(step_index)){
    // Use Polynomial interpolation
    local_time = time - get_double_support_t_start(step_index);
    dcm_vel_out = get_DCM_DS_vel_poly(step_index, local_time);
    // dcm_vel_out = get_DCM_DS_vel_minjerk(step_index, local_time);
  }else{ // Use exponential interpolation
    local_time = time - get_t_step_start(step_index);
    dcm_vel_out = get_DCM_vel_exp(step_index, local_time);
  }  


  // Discontinuous Interpolation
  // int step_index = which_step_index(time);
  // double local_time = time - get_t_step_start(step_index);
  // dcm_vel_out = get_DCM_vel_exp(step_index, local_time);
}

// Computes the reference virtual repellant point
void DCMWalkingReference::get_ref_r_vrp(const double t, Eigen::Vector3d & r_vrvp_out){
  Eigen::Vector3d dcm, dcm_vel;
  get_ref_dcm(t, dcm);
  get_ref_dcm_vel(t, dcm_vel);
  r_vrvp_out = dcm - b*dcm_vel;
}

void DCMWalkingReference::get_ref_reaction_force(const double t, Eigen::Vector3d & f_out){
  Eigen::Vector3d r_vrp_ref, com_pos_ref;
  get_ref_r_vrp(t, r_vrp_ref);
  get_ref_com(t, com_pos_ref);
  get_reaction_force(robot_mass, com_pos_ref, r_vrp_ref, f_out);  
}

int DCMWalkingReference::get_r_vrp_type(const int step_index){
  // clamp step index
  int index = clampINT(step_index, 0, rvrp_type_list.size() - 1);
  return rvrp_type_list[index];
}

int DCMWalkingReference::which_step_index(const double t){
  // clamp to 0.0
  if (t <= 0.0){
    return 0;
  }

  double t_exp_step_start = 0.0; // Double support starting time.
  double t_exp_step_end = 0.0; // step's exponential ending time.

  for (int i = 0; i < rvrp_list.size(); i++){
    t_exp_step_start = get_t_step_start(i);
    t_exp_step_end = get_t_step_end(i);
    if ((t_exp_step_start <= t) && (t <= t_exp_step_end)){
      return i;
    }
  }
  // the requested time is beyond so give the last step index
  return rvrp_list.size() - 1;


}


// Returns the step index to use given the input time from t_start.
int DCMWalkingReference::which_step_index_to_use(const double t){
  // clamp to 0.0
  if (t <= 0.0){
    return 0;
  }

  double t_ds_step_start = 0.0; // Double support starting time.
  double t_exp_step_end = 0.0; // step's exponential ending time.

  for (int i = 0; i < rvrp_list.size(); i++){
    t_ds_step_start = get_double_support_t_start(i);
    t_exp_step_end = get_t_step_end(i) - (alpha_ds*t_ds);
    if ((t_ds_step_start <= t) && (t <= t_exp_step_end)){
      return i;
    }
  }
  // the requested time is beyond so give the last step index
  return rvrp_list.size() - 1;
}


// If step index is an rvrp swing type, returns true and populates the value of swing start and end times
bool DCMWalkingReference::get_t_swing_start_end(const int step_index, double & swing_start_time, double & swing_end_time){
  if ((rvrp_type_list[step_index] == DCM_LL_SWING_VRP_TYPE) || (rvrp_type_list[step_index] == DCM_RL_SWING_VRP_TYPE)){
    swing_start_time = get_t_step_start(step_index) + t_ds*(1.0 - alpha_ds);
    swing_end_time = get_t_step_end(step_index) - (alpha_ds*t_ds);
    return true;
  }else{
    return false;
  }
}



int DCMWalkingReference::clampINT(int input, int low_bound, int upper_bound){
  if (input <= low_bound){
    return low_bound;
  }else if (input >= upper_bound){
    return upper_bound;
  }else{
    return input;
  }
}

double DCMWalkingReference::clampDOUBLE(double input, double low_bound, double upper_bound){
  if (input <= low_bound){
    return low_bound;
  }else if (input >= upper_bound){
    return upper_bound;
  }else{
    return input;
  }  
}

// returns the starting time of the step_index from t_start.
double DCMWalkingReference::get_t_step_start(const int step_index){
  // clamp step index
  int index = clampINT(step_index, 0, rvrp_list.size() - 1);

  // Accumulate duration of each step
  double t_step_start = 0.0;
  for (int i = 0; i < index; i++){
    t_step_start += get_t_step(i);
  }  
  return t_step_start;
}

double DCMWalkingReference::get_t_step_end(const int step_index){
  // clamp step index
  int index = clampINT(step_index, 0, rvrp_list.size() - 1);
  return get_t_step_start(index) + get_t_step(index);
}

// returns the double support starting time of the step_index from t_start.
double DCMWalkingReference::get_double_support_t_start(const int step_index){
  // clamp step index
  int index = clampINT(step_index, 0, rvrp_list.size() - 1);

  // First find initial starting time of the index from t_start
  double t_double_support_start = get_t_step_start(index);

  // Apply double support offset after the first step.
  if (step_index > 0){
    t_double_support_start -= (t_ds*alpha_ds);
  }

  return t_double_support_start;
}

// returns the double support ending time of the step_index from t_start.
double DCMWalkingReference::get_double_support_t_end(const int step_index){
  // clamp step index
  int index = clampINT(step_index, 0, rvrp_list.size() - 1);
  return get_double_support_t_start(index) + get_polynomial_duration(index);
}

void DCMWalkingReference::get_com_vel(const Eigen::Vector3d & com_pos, const Eigen::Vector3d & dcm, Eigen::Vector3d & com_vel_out){
  com_vel_out = (-1.0/b)*(com_pos - dcm);
}

void DCMWalkingReference::get_reaction_force(const double mass, const Eigen::Vector3d & com_pos, 
                                             const Eigen::Vector3d & r_vrp, Eigen::Vector3d fr_out){
  Eigen::Vector3d r_ecmp_c = (r_vrp - Eigen::Vector3d(0.0, 0.0, z_vrp));
  fr_out = (mass*gravity/z_vrp)*(com_pos - r_ecmp_c);
}

// computes the current r_vrp given 
//   -the DCM dynamics time constant b,
//  - the current dcm 
//  - and the current dcm_vel
void DCMWalkingReference::get_r_vrp(const double b_in, const Eigen::Vector3d & dcm, const Eigen::Vector3d & dcm_vel, Eigen::Vector3d & r_vrp_out){
  r_vrp_out = dcm - b_in*dcm_vel;
}



// computes the reference com trajectories by integration
void DCMWalkingReference::compute_reference_com(){
  // Do not process if rvrp list is empty
  if (rvrp_list.size() == 0){
    return;
  }

  compute_total_trajectory_time();
  double t_local = t_start;
  double t_local_end = t_start + t_end;

  // Compute discretization size
  int N_local = int(t_end/dt_local);

  // Resize reference vectors
  ref_com_pos.resize(N_local + 1);
  ref_com_vel.resize(N_local + 1);

  // Initialize variables
  Eigen::Vector3d com_pos = rvrp_list[0];
  Eigen::Vector3d com_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcm_cur = Eigen::Vector3d::Zero();

  for(int i = 0; i < (N_local + 1); i++){
    t_local = t_start + i*dt_local;
    get_ref_dcm(t_local, dcm_cur);
    get_com_vel(com_pos, dcm_cur, com_vel);
    com_pos = com_pos + com_vel*dt_local;

    // Set reference CoM position and velocities
    ref_com_pos[i] = com_pos;
    ref_com_vel[i] = com_vel;
  }
}

void DCMWalkingReference::get_ref_com(const double t, Eigen::Vector3d & com_out){
  double time = clampDOUBLE(t - t_start, 0.0, t_end);
  int index = int(time/dt_local);
  com_out = ref_com_pos[index];
}

void DCMWalkingReference::get_ref_com_vel(const double t, Eigen::Vector3d & com_vel_out){
  // Eigen::Vector3d com_pos, dcm;
  // get_ref_com(t, com_pos);
  // get_ref_dcm(t, dcm);
  // get_com_vel(com_pos, dcm, com_vel_out);
  // Velocities are zero before time starts 
  if (t < t_start){
    com_vel_out.setZero();
    return;
  }

  double time = clampDOUBLE(t - t_start, 0.0, t_end);
  int index = int(time/dt_local);
  com_vel_out = ref_com_vel[index];
}

void DCMWalkingReference::compute_reference_pelvis_ori(){
  pelvis_ori_quat_curves.clear();

  DracoFootstep prev_left_stance = initial_leftfoot_stance;
  DracoFootstep prev_right_stance = initial_rightfoot_stance;
  DracoFootstep stance_step;
  DracoFootstep target_step;
  DracoFootstep midfeet;

  // Initialize pelvis orientation
  midfeet.computeMidfeet(prev_left_stance, prev_right_stance, midfeet);
  Eigen::Quaterniond current_pelvis_ori = initial_ori; //midfeet.orientation;

  // Initialize the footstep counter
  int step_counter = 0;

  for(int i = 0; i < rvrp_type_list.size(); i++){
    // Swing State
    if ((rvrp_type_list[i] == DCM_RL_SWING_VRP_TYPE) || (rvrp_type_list[i] == DCM_LL_SWING_VRP_TYPE)){
      target_step = footstep_list[step_counter];
      if (target_step.robot_side == DRACO_LEFT_FOOTSTEP){
        stance_step = prev_right_stance;
        prev_left_stance = target_step;
      }else{
        stance_step = prev_left_stance;
        prev_right_stance = target_step;
      }
      // Find the midfeet 
      midfeet.computeMidfeet(stance_step, target_step, midfeet);
      // Create the hermite quaternion curve object
      pelvis_ori_quat_curves.push_back(HermiteQuaternionCurve(current_pelvis_ori, Eigen::Vector3d::Zero(),
                                                              midfeet.orientation, Eigen::Vector3d::Zero()));
      // Update the pelvis orientation
      current_pelvis_ori = midfeet.orientation;
      // Increment the step counter
      step_counter++;
    }else{
      // Orientation is constant during transfers
      midfeet.computeMidfeet(prev_left_stance, prev_right_stance, midfeet);
      current_pelvis_ori = midfeet.orientation;
      pelvis_ori_quat_curves.push_back(HermiteQuaternionCurve(current_pelvis_ori, Eigen::Vector3d::Zero(),
                                                              current_pelvis_ori, Eigen::Vector3d::Zero()));
    }
  }

}


void DCMWalkingReference::get_ref_ori_ang_vel_acc(const double t, Eigen::Quaterniond & quat_out,
                                                  Eigen::Vector3d & ang_vel_out,
                                                  Eigen::Vector3d & ang_acc_out){

  if (pelvis_ori_quat_curves.size() == 0){
    return;
  }

   // offset time and clamp. t_start is global start time.
  double time = clampDOUBLE(t - t_start, 0.0, t_end);
  int step_index = which_step_index(time);
  double t_traj_start = get_t_step_start(step_index);
  double t_traj_end = get_t_step(step_index);
  double traj_duration = t_traj_end - t_traj_start;

  // Clamp the time query for general VRP types that are still in transfer
  double time_query = clampDOUBLE(time, t_traj_start, t_traj_end);  
  // Initialize interpolation variable s.
  double s = (time_query - t_traj_start)/traj_duration;

  // If it is a swing step, update the trjaectory start times and end.
  if (get_t_swing_start_end(step_index, t_traj_start, t_traj_end)){
    // Clamp the time query for swing VRP types that are still in transfer
    time_query = clampDOUBLE(time, t_traj_start, t_traj_end);  
    // Update trajectory duration and interpolation variable
    traj_duration = t_traj_end - t_traj_start;
    s = (time_query - t_traj_start)/traj_duration;
  }

  // Obtain the reference values
  pelvis_ori_quat_curves[step_index].evaluate(s, quat_out);
  pelvis_ori_quat_curves[step_index].getAngularVelocity(s, ang_vel_out);
  pelvis_ori_quat_curves[step_index].getAngularAcceleration(s, ang_acc_out);

  // std::cout << "t:" << time << " s:" << s << " ang_vel_out = " << ang_vel_out.transpose() << std::endl;

}
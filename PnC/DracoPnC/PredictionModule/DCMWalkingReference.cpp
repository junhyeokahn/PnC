#include <PnC/DracoPnC/PredictionModule/DCMWalkingReference.hpp>

int const DCMWalkingReference::DCM_SWING_VRP_TYPE = 0;
int const DCMWalkingReference::DCM_TRANSFER_VRP_TYPE = 1;


DCMWalkingReference::DCMWalkingReference(){
    std::cout << "[DCMWalkingReference] Constructed" << std::endl;
} 

DCMWalkingReference::~DCMWalkingReference(){

}

// Sets the desired CoM Height
void DCMWalkingReference::setCoMHeight(double z_vrp_in){
  z_vrp = z_vrp_in;
} 

void DCMWalkingReference::setInitialTime(double t_start_in){
    t_start = t_start_in;
}


void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, 
                                                        const DracoFootstep & initial_footstance,
                                                        bool clear_list){
  // Store the input footstep list
  footstep_list = input_footstep_list;
  if (input_footstep_list.size() == 0){ 
    return;
  }
  // clear DCM variables if true
  if (clear_list){
    rvrp_list.clear();
    dcm_ini_list.clear();
    dcm_eos_list.clear(); 
    dcm_ini_DS_list.clear();
    dcm_vel_ini_DS_list.clear();
    dcm_end_DS_list.clear();
    dcm_vel_end_DS_list.clear();       
  }

  // Create an rvrp for the stance leg
  Eigen::Vector3d current_rvrp(0, 0, z_vrp); // From foot local frame
  Eigen::Vector3d current_stance_rvrp(0, 0, z_vrp); // The stance leg from the foot local frame
  Eigen::Vector3d left_stance_rvrp(0, 0, z_vrp); // a left stance leg from the foot local frame
  Eigen::Vector3d right_stance_rvrp(0, 0, z_vrp); // a right stance leg from the foot local frame

  current_stance_rvrp = initial_footstance.R_ori * current_stance_rvrp + initial_footstance.position;
  left_stance_rvrp = current_stance_rvrp;
  right_stance_rvrp = current_stance_rvrp;

  // Specify that this is the eos for the previous rvrp
  rvrp_type_list.clear();
  rvrp_type_list.push_back(DCM_TRANSFER_VRP_TYPE);
  dcm_eos_list.push_back(current_stance_rvrp);

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
    // Specify that this is the eos for the previous rvrp
    rvrp_type_list.push_back(DCM_SWING_VRP_TYPE);

    // Add this rvrp to the list and also populate the DCM states
    rvrp_list.push_back(current_rvrp);

    // Update previous_step side 
    previous_step = input_footstep_list[i].robot_side;
  }

  // Compute DCM states
  computeDCM_states();
}

void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & initial_footstance, const Eigen::Vector3d & initial_rvrp){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  rvrp_list.clear();
  dcm_ini_list.clear();
  dcm_eos_list.clear(); 
  dcm_ini_DS_list.clear();
  dcm_vel_ini_DS_list.clear();
  dcm_end_DS_list.clear();
  dcm_vel_end_DS_list.clear();

  // Add the initial virtual repellant point. 
  rvrp_list.push_back(initial_rvrp); 

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
  }else if (rvrp_type_list[step_i] == DCMWalkingReference::DCM_SWING_VRP_TYPE){
    return t_ss + t_ds; // every swing has a double support transfer
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
  dcm_ini_DS_list.clear(); dcm_vel_ini_DS_list.clear(); 
  dcm_end_DS_list.clear(); dcm_vel_end_DS_list.clear();  
  dcm_P.clear();

  // DCM ini and eos list is one size less than the RVRP list
  dcm_ini_list.resize(rvrp_list.size() - 1);
  dcm_eos_list.resize(rvrp_list.size() - 1);

  // DS DCM list is equal to the size of the rvrp  list
  dcm_ini_DS_list.resize(rvrp_list.size()); dcm_vel_ini_DS_list.resize(rvrp_list.size()); 
  dcm_end_DS_list.resize(rvrp_list.size()); dcm_vel_end_DS_list.resize(rvrp_list.size());  

  // Use backwards recursion to compute the initial and final dcm states
  double t_step = 0.0;
  for (int i = rvrp_list.size()-2; i >= 0; i--){
    // Get the t_step to use for backwards integration
    t_step = get_t_step(i);
    // Compute dcm_ini for step i
    dcm_ini_list[i] = computeDCM_ini_i(rvrp_list[i], t_step, dcm_eos_list[i]);

    // Set dcm_eos for step i-1
    if (i > 0){
      dcm_eos_list[i-1] = dcm_ini_list[i];
    }
  }
  // Last element of the DCM end of step list is equal to the last rvrp.
  dcm_eos_list.back() = rvrp_list.back();

  // Find boundary conditions for the Polynomial interpolator
  for (int i = 0; rvrp_list.size(); i++){
   // compute boundary conditions
   dcm_ini_DS_list[i] = computeDCM_iniDS_i(i, alpha_ds*t_ds);
   dcm_end_DS_list[i] = computeDCM_eoDS_i(i, (1.0-alpha_ds)*t_ds);
   dcm_vel_ini_DS_list[i] = computeDCMvel_iniDS_i(i, alpha_ds*t_ds);
   dcm_vel_end_DS_list[i] = computeDCMvel_eoDS_i(i, (1.0-alpha_ds)*t_ds);
  }

  // compute polynomial interpolator matrix
  double Ts = t_ds; // set transfer duration time
  for (int i = 0; rvrp_list.size(); i++){
    Ts = get_polynomial_duration(i);
    dcm_P[i] = polynomialMatrix(Ts, dcm_ini_DS_list[i], dcm_vel_ini_DS_list[i],
                                      dcm_end_DS_list[i], dcm_vel_end_DS_list[i]);
  }

  // Compute the total trajectory time.
  compute_total_trajectory_time();
}

void DCMWalkingReference::compute_total_trajectory_time(){
  t_end = 0.0;
  for (int i = 0; rvrp_list.size(); i++){
    t_end += get_t_step(i);
  }
}

double DCMWalkingReference::get_polynomial_duration(const int step_index){
  // first step has polynomial duration of only ending double support
  if (step_index == 0){
    return (1.0-alpha_ds)*t_ds;
  } // last step only has polynomial duration of the initial double support phase
  else if (step_index == (rvrp_list.size() - 1)){
    return alpha_ds*t_ds;
  }
  return t_ds;
}

Eigen::Vector3d DCMWalkingReference::computeDCM_iniDS_i(const int & step_index, const double t_DS_ini){
  // Set Boundary condition. First element of eoDS is equal to the first element of the rvrp list
  if (step_index == 0){
    return rvrp_list.front();
  }
  return rvrp_list[step_index - 1] + std::exp(-t_DS_ini/b) * (dcm_ini_list[step_index] - rvrp_list[step_index - 1]);
}

Eigen::Vector3d DCMWalkingReference::computeDCM_eoDS_i(const int & step_index, const double t_DS_end){
  // Set Boundary condition. Last element of eoDS is equal to the last element of the rvrp list
  if (step_index == (rvrp_list.size() - 1)){
    return rvrp_list.back();
  }
  return rvrp_list[step_index] + std::exp(t_DS_end/b) * (dcm_ini_list[step_index] - rvrp_list[step_index]);
}

Eigen::Vector3d DCMWalkingReference::computeDCMvel_iniDS_i(const int & step_index, const double t_DS_ini){
  // Set Boundary condition. Velocities at the very beginning and end are always 0.0
  if (step_index == 0){
    return Eigen::Vector3d::Zero();
  }
  return (-1.0/b)*std::exp(-t_DS_ini/b) * (dcm_ini_list[step_index] - rvrp_list[step_index - 1]);
}

Eigen::Vector3d DCMWalkingReference::computeDCMvel_eoDS_i(const int & step_index, const double t_DS_end){
  // Set Boundary condition. Velocities at the very beginning and end are always 0.0
  if (step_index == (rvrp_list.size() - 1)){
    return Eigen::Vector3d::Zero();
  }
  return (1.0/b)*std::exp(t_DS_end/b) * (dcm_ini_list[step_index] - rvrp_list[step_index]);
}


Eigen::MatrixXd DCMWalkingReference::polynomialMatrix(const double Ts,
                                                      const Eigen::Vector3d & dcm_ini, const Eigen::Vector3d & dcm_vel_ini,
                                                      const Eigen::Vector3d & dcm_end, const Eigen::Vector3d & dcm_vel_end){

  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(4,4);

  // Construct matrix mat
  mat(0,0) = 2.0/std::pow(Ts, 3);  mat(0,1) = 1.0/std::pow(Ts, 2); mat(0,2) = -2.0/std::pow(Ts, 3); mat(0,3) = 1.0/std::pow(Ts, 2);
  mat(1,0) = -3.0/std::pow(Ts, 2); mat(1,1) = -2.0/Ts;             mat(1,2) = 3.0/std::pow(Ts, 2);  mat(1,3) = -1.0/Ts;
                                   mat(2,1) = 1.0;

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


// Returns the DCM double support polynomial interpolation for the requested step_index.
// time, t, is clamped between 0.0 and t_step.
Eigen::Vector3d DCMWalkingReference::get_DCM_DS_poly(const int & step_index, const double & t){
  double Ts = get_polynomial_duration(step_index);
  double time = clampDOUBLE(t, 0.0, Ts);

  Eigen::MatrixXd t_mat = Eigen::MatrixXd::Zero(1,4);
  t_mat(0,0) = std::pow(time, 3);  t_mat(0,1) = std::pow(time, 2);  t_mat(0,2) = time; t_mat(0,3) = 1.0; 
  return t_mat*dcm_P[step_index];
}

// Returns the DCM double support velocity polynomial interpolation for the requested step_index.
// time, t, is clamped between 0.0 and t_step.
Eigen::Vector3d DCMWalkingReference::get_DCM_DS_vel_poly(const int & step_index, const double & t){
  double Ts = get_polynomial_duration(step_index);
  double time = clampDOUBLE(t, 0.0, Ts);

  Eigen::MatrixXd t_mat = Eigen::MatrixXd::Zero(1,4);
  t_mat(0,0) = 3.0*std::pow(time, 2);  t_mat(0,1) = 2.0*t;  t_mat(0,2) = 1.0; 
  return t_mat*dcm_P[step_index];
}


void DCMWalkingReference::get_ref_dcm(const double t, Eigen::Vector3d & dcm_out){
  // offset time and clamp. t_start is global start time.
  double time = clampDOUBLE(t - t_start, 0.0, t_end);
  
  // interpolation index to use
  int step_index = which_step_index_to_use(time);  
  double local_time;
  // Use Polynomial interpolation
  if (time <= get_double_support_t_end(step_index)){
    local_time = time - get_double_support_t_start(step_index);
    dcm_out = get_DCM_DS_poly(step_index, local_time);
  }else{ // Use exponential interpolation
    local_time = time - get_t_step_start(step_index);
    dcm_out = get_DCM_exp(step_index, local_time);
  }

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

  // Apply double support offset after the first. All steps after the first step.
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


/*
void WalkingPatternGenerator::compute_com_dcm_trajectory(const Eigen::Vector3d & initial_com){
  Eigen::Vector3d com_pos = initial_com;
  Eigen::Vector3d dcm_pos; dcm_pos.setZero();
  int step_index = 0;
  double t = 0.0;
  double t_step = get_t_step(step_index);
  double t_prev = 0.0;
  double dt = internal_dt;

  // Always try to compute CoM finely. Also, ensure that we follow the desired discretization
  double dt_local = 1e-3; // Use this discretization for integrating the CoM
  int N_local = int(get_total_trajectory_time()/dt_local);
  int j = 0;

  // In case N_size is larger than N_local: 
  if (N_size > N_local){
    // Compute using N_size as the discretization. This is rarely the case
    for(int i = 0; i < N_size; i++){
      // x_post = dx*dt + x_pre
      t = dt*i;
      com_pos = get_com_vel(com_pos, step_index, t-t_prev)*dt + com_pos;
      dcm_pos = get_desired_DCM(step_index, t-t_prev);
      // Check if t-t_prev exceeded the current t_step and if we can increment the step index
      if ( ((t-t_prev) >= t_step) && (step_index < rvrp_type_list.size()-1) ){
        step_index++;
        t_prev = t;
        t_step = get_t_step(step_index);        
      }

        // std::cout << com_pos.transpose() << std::endl;
        // Store the CoM position
        traj_pos_com.set_pos(i, com_pos);
        traj_dcm_pos.set_pos(i, dcm_pos);  
    }
  }else{
    // Compute with a more fine integration of the CoM. This is usually the case
    for(int i = 0; i < N_local; i++){
      if (j < N_size){
        // x_post = dx*dt + x_pre
        t = dt_local*i;
        com_pos = get_com_vel(com_pos, step_index, t-t_prev)*dt_local + com_pos;
        dcm_pos = get_desired_DCM(step_index, t-t_prev);
        // Check if t-t_prev exceeded the current t_step and if we can increment the step index
        if ( ((t-t_prev) >= t_step) && (step_index < rvrp_type_list.size()-1) ){
          step_index++;
          t_prev = t;
          t_step = get_t_step(step_index);        
        }

        // Store the COM position at the desired discretization
        if (i % (N_local/N_size) == 0){
          // std::cout << com_pos.transpose() << std::endl;
          traj_pos_com.set_pos(j, com_pos);
          traj_dcm_pos.set_pos(j, dcm_pos);
          j++;      
        }
      }
      else{
        break;
      }
    }
  }
}
*/
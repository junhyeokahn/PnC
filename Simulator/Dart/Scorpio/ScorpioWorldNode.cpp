#include <Simulator/Dart/Scorpio/ScorpioWorldNode.hpp>
#include <cmath>

ScorpioWorldNode::ScorpioWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world),
      count_(0),
      t_(0.0),
      servo_rate_(0.001){
    world_ = _world;
    mSkel_ = world_->getSkeleton("Scorpio_Kin");
    mGround_ = world_->getSkeleton("ground_skeleton");
    mDof_ = mSkel_->getNumDofs();
    mActuatingDof_ = mDof_ - 4;
    q_init_.resize(mDof_);
    q_init_ = mSkel_->getPositions();
    joint_idx_.resize(mActuatingDof_);
    joint_idx_[0] = mSkel_->getDof("joint1")->getIndexInSkeleton();
    joint_idx_[1] = mSkel_->getDof("joint2")->getIndexInSkeleton();
    joint_idx_[2] = mSkel_->getDof("joint5")->getIndexInSkeleton();
    joint_idx_[3] = mSkel_->getDof("joint6")->getIndexInSkeleton();
    joint_idx_[4] = mSkel_->getDof("joint9")->getIndexInSkeleton();
    joint_idx_[5] = mSkel_->getDof("joint10")->getIndexInSkeleton();
    joint_idx_[6] = mSkel_->getDof("joint11")->getIndexInSkeleton();

    YAML::Node simulation_cfg = 
         YAML::LoadFile(THIS_COM "Config/Scorpio/SIMULATION.yaml");
    // 0: servo, 1: force
    myUtils::readParameter(simulation_cfg, "actuator_type", actuator_type_);

    YAML::Node control_cfg = 
        YAML::LoadFile(THIS_COM "Config/Scorpio/CONTROL.yaml");
    myUtils::readParameter(control_cfg, "Amp", Amp_ );
    myUtils::readParameter(control_cfg, "Freq", Freq_);
    myUtils::readParameter(control_cfg, "sim_case", sim_case_);
    myUtils::readParameter(control_cfg, "kp", kp_);
    myUtils::readParameter(control_cfg, "kv", kv_);
    myUtils::readParameter(control_cfg, "upper_limit", q_limit_u_);
    myUtils::readParameter(control_cfg, "lower_limit", q_limit_l_);
    myUtils::readParameter(control_cfg, "dq_input", dq_input_);
    myUtils::readParameter(control_cfg, "dx_input", dx_input_);

    std::cout<<"amp: "<<Amp_<<std::endl;
    std::cout<<"freq: "<< Freq_<<std::endl;
    std::cout<<"sim_case: "<<sim_case_ <<std::endl;
    if(actuator_type_ == 0)
        std::cout<<"actuator_type: servo "<<std::endl;
    else
        std::cout<<"actuator_type: force "<<std::endl;
    
    std::cout<<"upper: " <<q_limit_u_ <<std::endl;
    std::cout<<"lower: " <<q_limit_l_ <<std::endl;
    q_limit_u_ = q_limit_u_/180.0*M_PI;
    q_limit_l_ = q_limit_l_/180.0*M_PI;

    active1 = mSkel_->getJoint("joint1");
    active2 = mSkel_->getJoint("joint2");
    active3 = mSkel_->getJoint("joint5");
    active4 = mSkel_->getJoint("joint6");
    active5 = mSkel_->getJoint("joint9");
    active6 = mSkel_->getJoint("joint10");
    active7 = mSkel_->getJoint("joint11");
}


ScorpioWorldNode::~ScorpioWorldNode() {

}

void ScorpioWorldNode::GetActiveJointInfo(Eigen::VectorXd & cur_pos, Eigen::VectorXd & cur_vel){
    Eigen::VectorXd j_pos(mDof_);
    Eigen::VectorXd j_vel(mDof_);
    j_pos = mSkel_->getPositions();
    j_vel = mSkel_->getVelocities();

    cur_pos.resize(mActuatingDof_);
    cur_pos.setZero();
    cur_vel.resize(mActuatingDof_);
    cur_vel.setZero();

    for(int i(0); i< mActuatingDof_; ++i){
        cur_pos[i] = j_pos[joint_idx_[i]];
        cur_vel[i] = j_vel[joint_idx_[i]];
    }
}

void ScorpioWorldNode::SetActivePosition(Eigen::VectorXd & des_pos){
    for(int i(0); i<mActuatingDof_; ++i)
        mSkel_->setPosition(joint_idx_[i], des_pos[i]);
}

void ScorpioWorldNode::SetActiveForce(Eigen::VectorXd & des_force){
    Eigen::VectorXd cmd_force(mDof_);
    cmd_force.setZero();

    for( int i(0); i< mActuatingDof_; ++i){
        cmd_force[joint_idx_[i]]  = des_force[i];
    }

    mSkel_->setForces(cmd_force);
}

void ScorpioWorldNode::UpdateSystem(Eigen::VectorXd & q_activate){
    Eigen::VectorXd q_full(mDof_);
    
    q_full[0] = q_activate[0];
    q_full[1] = q_activate[1];
    q_full[2] = -q_activate[1];
    q_full[3] = q_activate[1];
    q_full[4] = q_activate[2];
    q_full[5] = q_activate[3];
    q_full[6] = -q_activate[3];
    q_full[7] = q_activate[3];
    q_full[8] = q_activate[4];
    q_full[9] = q_activate[5];
    q_full[10] = q_activate[6];

    mSkel_->setPositions(q_full);
    mSkel_->computeForwardKinematics(true, false, false);
}

void ScorpioWorldNode::SetActiveVelocity(Eigen::VectorXd & des_vel){

    active1->setCommand(0,des_vel[0]);
    active2->setCommand(0,des_vel[1]);
    active3->setCommand(0,des_vel[2]);
    active4->setCommand(0,des_vel[3]);
    active5->setCommand(0,des_vel[4]);
    active6->setCommand(0,des_vel[5]);
    active7->setCommand(0,des_vel[6]);
}

void ScorpioWorldNode::ComputeWorkspace(double rad_interval){
    Eigen::VectorXd delta(mActuatingDof_);
    delta = q_limit_u_ - q_limit_l_;
    Eigen::VectorXd q_update(mActuatingDof_);
    q_update = q_limit_u_;
    Eigen::Vector3d end_pos;
    end_pos.setZero();

    // total number of iteration
    int total_iter = 1.0;
    Eigen::VectorXd Num_iter(mActuatingDof_);
    for(int i(0); i< mActuatingDof_; ++i){
        Num_iter[i] = (int)(delta[i]/rad_interval)+1;
    }

    for(int i(0); i<3; ++i){
        total_iter = total_iter*Num_iter[i];
    }

    myUtils::pretty_print(Num_iter,std::cout, "number_iter");
    PrepareWorkspaceAnalysis(dx_input_);
    Eigen::Tensor<double, 3> WorkspaceMap(num_cart_[0], num_cart_[1], num_cart_[2]);
    WorkspaceMap.setZero();
    
    Eigen::Vector3d offset(0, 0.120, 0);
    
    std::cout<<"*****[start to iterate the workspace]********"<<std::endl;
    // start iteration
    int cur_iter = 0;
    for(int j1(0); j1<Num_iter[0]; j1++){
        q_update[0] = q_limit_l_[0] + j1*rad_interval;

        for(int j2(0); j2<Num_iter[1]; j2++){
            q_update[1] = q_limit_l_[1] + j2*rad_interval;
            
            for(int j3(0); j3<Num_iter[2]; j3++){
                q_update[2] = q_limit_l_[2] + j3*rad_interval;

                for(int j4(0); j4< Num_iter[3]; j4++){
                    q_update[3] = q_limit_l_[3] + j4*rad_interval;
                   
                    for(int j5(0); j5< Num_iter[4]; j5++ ){
                        q_update[4] = q_limit_l_[4] + j5*rad_interval;
                        
                        for(int j6(0); j6<Num_iter[5]; j6++){
                                q_update[5] = q_limit_l_[5] + j6*rad_interval;

                                for(int j7(0); j7<Num_iter[6]; j7++ ){
                                    q_update[6] = q_limit_l_[6] + j7*rad_interval;
                                    UpdateSystem(q_update);
                                    end_pos = mSkel_->getBodyNode("link11")->getTransform()*offset;
                                    UpdateWorkspace(end_pos, WorkspaceMap);
                                }
                        }
                    }
                }
           cur_iter = cur_iter + 1;   
           std::cout<<"["<<cur_iter<<"/"<<total_iter<<"] Iteration step."<<std::endl;  

           }
        }
    }

    std::cout<<"*****[start to save the results]********"<<std::endl;
    int total_save = num_cart_[1]*num_cart_[2];
    int cur_save = 0;
    // save the Tensor
    Eigen::VectorXd res(num_cart_[0]);
    res.setZero();
    for( int i(0); i<num_cart_[2]; ++i ){
        for (int j(0); j<num_cart_[1]; ++j){
            for (int k(0); k<num_cart_[0]; ++k){
                res[k] = WorkspaceMap(k,j,i);
            }
            myUtils::saveVector(res,"work_count");
            cur_save = cur_save+1;
        }
    std::cout<<"["<<cur_save <<"/"<<total_save<<"] The results are saved."<<std::endl;
    }
}

void ScorpioWorldNode::PrepareWorkspaceAnalysis(double dx){
    YAML::Node control_cfg = 
        YAML::LoadFile(THIS_COM "Config/Scorpio/CONTROL.yaml");
    myUtils::readParameter(control_cfg, "upper_cart", upper_cart_);
    myUtils::readParameter(control_cfg, "lower_cart", lower_cart_);

    Eigen::VectorXd num_box = (upper_cart_ - lower_cart_)/dx;
    num_cart_.resize(3);
    num_cart_.setZero();
    num_cart_[0] = (int)num_box[0];
    num_cart_[1] = (int)num_box[1];
    num_cart_[2] = (int)num_box[2];

    std::cout<<"num_cart: "<< num_cart_<<std::endl;
    delta_cart_ = dx;
}

void ScorpioWorldNode::UpdateWorkspace(Eigen::Vector3d & pos, Eigen::Tensor<double, 3> & Workcount){
    Eigen::VectorXd pos_offset(3);
    pos_offset.setZero();

    pos_offset[0] = pos[0] - lower_cart_[0];
    pos_offset[1] = pos[1] - lower_cart_[1];
    pos_offset[2] = pos[2] - lower_cart_[2];

    int room_x = floor(pos_offset[0]/delta_cart_);
    int room_y = floor(pos_offset[1]/delta_cart_);
    int room_z = floor(pos_offset[2]/delta_cart_);

    Workcount(room_x, room_y, room_z) = Workcount(room_x, room_y, room_z) + 1.0;

}

void ScorpioWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;
    std::cout<<"time: "<<t_<<std::endl;

    Eigen::VectorXd pos_des(mActuatingDof_);
    Eigen::VectorXd pos_cur(mActuatingDof_);
    Eigen::VectorXd vel_cur(mActuatingDof_);
    Eigen::VectorXd torque_des(mActuatingDof_);

    GetActiveJointInfo(pos_cur, vel_cur);
    switch(sim_case_){
        case 0: {
            pos_des[0] = q_init_[joint_idx_[0]]+0.;
            pos_des[1] = q_init_[joint_idx_[1]]+Amp_[1]/180.0*M_PI*sin((2*M_PI)*Freq_[1]*t_);
            pos_des[2] = q_init_[joint_idx_[2]]+0.;
            pos_des[3] = q_init_[joint_idx_[3]]+Amp_[3]/180.0*M_PI*sin((2*M_PI)*Freq_[3]*t_);
            pos_des[4] = q_init_[joint_idx_[4]]+Amp_[4]/180.0*M_PI*sin((2*M_PI)*Freq_[4]*t_);
            pos_des[5] = q_init_[joint_idx_[5]]+Amp_[5]/180.0*M_PI*sin((2*M_PI)*Freq_[5]*t_);
            pos_des[6] = q_init_[joint_idx_[6]]+Amp_[6]/180.0*M_PI*sin((2*M_PI)*Freq_[6]*t_);
            break;
        }
        case 1:{
            pos_des[0] = q_init_[joint_idx_[0]]+Amp_[0]/180.0*M_PI*sin(Freq_[0]*(2*M_PI)*t_);
            pos_des[1] = q_init_[joint_idx_[1]]+0.;
            pos_des[2] = q_init_[joint_idx_[2]]-Amp_[2]/180.0*M_PI*sin(Freq_[2]*((2*M_PI)*t_-M_PI/2))-Amp_[2]/180.0*M_PI;
            pos_des[3] = q_init_[joint_idx_[3]]+0.;
            pos_des[4] = q_init_[joint_idx_[4]]+Amp_[4]/180.0*M_PI*sin((2*M_PI)*Freq_[4]*t_);
            pos_des[5] = q_init_[joint_idx_[5]]+Amp_[5]/180.0*M_PI*sin((2*M_PI)*Freq_[5]*t_);
            pos_des[6] = q_init_[joint_idx_[6]]+Amp_[6]/180.0*M_PI*sin((2*M_PI)*Freq_[6]*t_);
            break;
        }
        case 2:{
            pos_des[0] = q_init_[joint_idx_[0]]+Amp_[0]/180.0*M_PI*sin(Freq_[0]*(2*M_PI)*t_);
            pos_des[1] = q_init_[joint_idx_[1]]+Amp_[1]/180.0*M_PI*sin((2*M_PI)*Freq_[1]*t_);
            pos_des[2] = q_init_[joint_idx_[2]]-Amp_[2]/180.0*M_PI*sin((2*M_PI)*Freq_[2]*t_);
            pos_des[3] = q_init_[joint_idx_[3]]+Amp_[3]/180.0*M_PI*sin((2*M_PI)*Freq_[3]*t_);
            pos_des[4] = q_init_[joint_idx_[4]]+Amp_[4]/180.0*M_PI*sin((2*M_PI)*Freq_[4]*t_);
            pos_des[5] = q_init_[joint_idx_[5]]+Amp_[5]/180.0*M_PI*sin((2*M_PI)*Freq_[5]*t_);
            pos_des[6] = q_init_[joint_idx_[6]]+Amp_[6]/180.0*M_PI*sin((2*M_PI)*Freq_[6]*t_);
            break;
        }
        default:
            pos_des.setZero();
    }

 
    switch(actuator_type_ ){
        case 0:{
            Eigen::VectorXd vel_des(mActuatingDof_);
            vel_des.setZero();

            vel_des = (pos_des- pos_cur)/servo_rate_;
            SetActiveVelocity(vel_des);
            break;
        }
        case 1:{     
            for( int i(0); i< mActuatingDof_; ++i )
                torque_des[i] = kp_[i]*(pos_des[i]-pos_cur[i]) - kv_[i]*(vel_cur[i]);
            
            SetActiveForce(torque_des);
            break;
        }
        default:{
            UpdateSystem(pos_des);
            break;
        }
    }

    
    // double interval(0.0174533);
    double interval = dq_input_;
    ComputeWorkspace(interval); 
    count_++;
    
    if(count_ > 0){
        std::cout<<"Workspace Analysis finished! "<<std::endl;
        while(1);
    }
}


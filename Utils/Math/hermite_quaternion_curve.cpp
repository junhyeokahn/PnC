#include <Utils/Math/hermite_quaternion_curve.hpp>

/*
An implementation of the Hermite Quaternion Curve from:
Kim, Myoung-Jun, Myung-Soo Kim, and Sung Yong Shin. 
"A general construction scheme for unit quaternion curves with simple high order derivatives." SIGGRAPH. Vol. 95. 1995.
*/

// Note that we implement the global frame case not the local frame!

HermiteQuaternionCurve::HermiteQuaternionCurve(const Eigen::Quaterniond & quat_start, const Eigen::Vector3d & angular_velocity_start,
                                               const Eigen::Quaterniond & quat_end, const Eigen::Vector3d & angular_velocity_end){
  qa = quat_start;
  omega_a = angular_velocity_start;

  qb = quat_end;
  omega_b = angular_velocity_end;

  s_ = 0.0;
  initialize_data_structures();
  // std::cout << "[Hermite Quaternion Curve] Initialized" << std::endl;
}

HermiteQuaternionCurve::~HermiteQuaternionCurve(){}

void HermiteQuaternionCurve::initialize_data_structures(){
  q0 = qa;

  if (omega_a.norm() < 1e-6){
    q1 = qa*Eigen::Quaterniond(1, 0, 0, 0);    
  }else{
    q1 = qa*Eigen::Quaterniond( Eigen::AngleAxisd( omega_a.norm()/3.0, omega_a/omega_a.norm() ) ); // q1 = qa*exp(wa/3.0)    
  }

  if (omega_b.norm() < 1e-6){
    q2 = qb*Eigen::Quaterniond(1, 0, 0, 0);    
  }else{  
    q2 = qb*Eigen::Quaterniond( Eigen::AngleAxisd( omega_b.norm()/3.0, -omega_b/omega_b.norm() ) ); // q2 = qb*exp(wb/3.0)^-1
  }

  q3 = qb;


  // std::cout << "q0" << std::endl;
  // printQuat(q0);
  // std::cout << "q1" << std::endl;
  // printQuat(q1);
  // std::cout << "q2" << std::endl;
  // printQuat(q2);  
  // std::cout << "q3" << std::endl;
  // printQuat(q3);

  // for world frame angular velocities, do: q_1*q_0.inverse(). for local frame do: q_0.inverse()*q_1

  // Global Frame
  omega_1aa = q1*q0.inverse();
  omega_2aa = q2*q1.inverse();
  omega_3aa = q3*q2.inverse();

  // Local Frame:
  // omega_1aa = q0.inverse()*q1;
  // omega_2aa = q1.inverse()*q2;
  // omega_3aa = q2.inverse()*q3;

  omega_1 = omega_1aa.axis() * omega_1aa.angle(); 
  omega_2 = omega_2aa.axis() * omega_2aa.angle();  
  omega_3 = omega_3aa.axis() * omega_3aa.angle();

  // std::cout << "omega_1:" << omega_1.transpose() << std::endl;
  // std::cout << "omega_2:" << omega_2.transpose() << std::endl;
  // std::cout << "omega_3:" << omega_3.transpose() << std::endl;
}

void HermiteQuaternionCurve::computeBasis(const double & s_in){
  s_ = this->clamp(s_in);
  b1 = 1 - std::pow((1-s_),3);
  b2 = 3*std::pow(s_, 2) - 2*std::pow((s_), 3);
  b3 = std::pow(s_, 3);

  bdot1 = 3*std::pow((1-s_),2);
  bdot2 = 6*s_ - 6*std::pow((s_), 2);
  bdot3 = 3*std::pow((s_), 2);

  bddot1 = -6*(1-s_);
  bddot2 = 6 - 12*s_;
  bddot3 = 6*s_;
}

void HermiteQuaternionCurve::evaluate(const double & s_in, Eigen::Quaterniond & quat_out){
  s_ = this->clamp(s_in);  
  computeBasis(s_);

  qtmp1 = Eigen::AngleAxisd(omega_1aa.angle()*b1, omega_1aa.axis());
  qtmp2 = Eigen::AngleAxisd(omega_2aa.angle()*b2, omega_2aa.axis());
  qtmp3 = Eigen::AngleAxisd(omega_3aa.angle()*b3, omega_3aa.axis());

  // quat_out = q0*qtmp1*qtmp2*qtmp3; // local frame
  quat_out = qtmp3*qtmp2*qtmp1*q0; // global frame
}

void HermiteQuaternionCurve::getAngularVelocity(const double & s_in, Eigen::Vector3d & ang_vel_out){
  // world frame: w(t) = qdot(t)*q^-1(t)
  // local frame: w(t) = q^-1(t)*qdot(t)
  // s_ = this->clamp(s_in);  
  // computeBasis(s_);

  // qtmp1 = Eigen::AngleAxisd(omega_1aa.angle()*b1, omega_1aa.axis());
  // qtmp2 = Eigen::AngleAxisd(omega_2aa.angle()*b2, omega_2aa.axis());
  // qtmp3 = Eigen::AngleAxisd(omega_3aa.angle()*b3, omega_3aa.axis());

  // Eigen::Quaterniond q1dot; q1dot.vec() = omega_1*bdot1; q1dot *= qtmp1;
  // Eigen::Quaterniond q2dot; q2dot.vec() = omega_2*bdot2; q2dot *= qtmp2;
  // Eigen::Quaterniond q3dot; q3dot.vec() = omega_3*bdot3; q3dot *= qtmp3;

  // // Computing global frame angular velocity
  // Eigen::Quaterniond quat_dot;
  // Eigen::Quaterniond quat_out = qtmp3*qtmp2*qtmp1*q0; // global frame

  // quat_dot.vec() = (q3dot*(qtmp2*qtmp1*q0)).vec() + (q2dot*(qtmp3*qtmp1*q0)).vec() + (q1dot*(qtmp3*qtmp2*q0)).vec();
  // quat_dot.w() = (q3dot*(qtmp2*qtmp1*q0)).w() + (q2dot*(qtmp3*qtmp1*q0)).w() + (q1dot*(qtmp3*qtmp2*q0)).w();

  // ang_vel_out = (quat_dot*quat_out.conjugate()).vec(); //omega_1*b1 + omega_2 * b2 + omega_3 * b3;

  s_ = this->clamp(s_in);  
  computeBasis(s_);
  ang_vel_out = omega_1*bdot1 + omega_2 * bdot2 + omega_3 * bdot3;

}

// For world frame
void HermiteQuaternionCurve::getAngularAcceleration(const double & s_in, Eigen::Vector3d & ang_acc_out){
  // world frame: a(t) = (qddot(t)*q^-1(t) - (qdot(t)*q^-1(t))^2)
  // local frame: a(t) = (q^-1(t)*qddot(t) - (q^-1(t)*qdot(t))^2)


  // Numerically differentiate since we know the angular velocity
  // bool forward_diff = true;
  // if (s_ > 0.5){
  //   forward_diff = false;
  // }

  // double ds = 1e-6;
  // double s_query = s_in;
  // Eigen::Vector3d ang_vel_1, ang_vel_2;
  // if (forward_diff){
  //   s_query = s_in + ds;  
  //   getAngularVelocity(s_query, ang_vel_2);
  //   s_query = s_in;       
  //   getAngularVelocity(s_query, ang_vel_1);
  // }else {
  //   s_query = s_in;       
  //   getAngularVelocity(s_query, ang_vel_2);
  //   s_query = s_in - ds;  
  //   getAngularVelocity(s_query, ang_vel_1);
  // }
  // ang_acc_out = (ang_vel_2 - ang_vel_1)/ds;

  s_ = this->clamp(s_in);  
  computeBasis(s_);
  ang_acc_out = omega_1*bddot1 + omega_2*bddot2 + omega_3*bddot3;
}


double HermiteQuaternionCurve::clamp(const double & s_in, double lo, double hi){
    if (s_in < lo){
        return lo;
    }
    else if(s_in > hi){
        return hi;
    }else{
        return s_in;
    }

}

void HermiteQuaternionCurve::printQuat(const Eigen::Quaterniond & quat){
  std::cout <<  quat.x() << " " <<
                quat.y() << " " <<
                quat.z() << " " <<
                quat.w() << " " << std::endl;
}
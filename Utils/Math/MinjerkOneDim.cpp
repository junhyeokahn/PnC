#include <Utils/Math/MinjerkOneDim.hpp>

// Empty Constructor
MinJerk_OneDimension::MinJerk_OneDimension(){
    Initialization();
}
// Parameter Constructor
MinJerk_OneDimension::MinJerk_OneDimension(const Eigen::Vector3d & init, const Eigen::Vector3d & end,
        const double & time_start, const double & time_end){
    Initialization();
    setParams(init, end, time_start, time_end);
}

//Destructor
MinJerk_OneDimension::~MinJerk_OneDimension(){}

void MinJerk_OneDimension::Initialization(){
    // Initialize to the corrrect sizes
    C_mat = Eigen::MatrixXd::Zero(6,6);
    C_mat_inv = Eigen::MatrixXd::Zero(6,6);
    a_coeffs = Eigen::VectorXd::Zero(6);
    bound_cond = Eigen::VectorXd::Zero(6);

    init_cond = Eigen::VectorXd::Zero(3);
    end_cond = 	Eigen::VectorXd::Zero(3);
    to = 0.0;
    tf = 1.0;
}

void MinJerk_OneDimension::setParams(const Eigen::Vector3d & init, const Eigen::Vector3d & end,
        const double & time_start, const double & time_end){
    // Set the Parameters
    init_cond = init;
    end_cond = end;
    bound_cond.head(3) = init_cond;
    bound_cond.tail(3) = end_cond;
    to = time_start;
    tf = time_end;

    // Construct C matrix
    C_mat(0,0) = 1.0; C_mat(0,1) = to;  C_mat(0,2) = std::pow(to, 2); C_mat(0,3) = std::pow(to, 3);     C_mat(0,4) = std::pow(to, 4)     ; C_mat(0,5) = std::pow(to, 5);
    C_mat(1,1) = 1.0; C_mat(1,2) = 2.0*to         ; C_mat(1,3) = 3.0*std::pow(to, 2); C_mat(1,4) = 4.0*std::pow(to, 3) ; C_mat(1,5) = 5.0*std::pow(to, 4);
    C_mat(2,2) = 2.0            ; C_mat(2,3) = 6.0*to             ; C_mat(2,4) = 12.0*std::pow(to, 2); C_mat(2,5) = 20.0*std::pow(to, 3);
    C_mat(3,0) = 1.0; C_mat(3,1) = tf;  C_mat(3,2) = std::pow(tf, 2); C_mat(3,3) = std::pow(tf, 3);     C_mat(3,4) = std::pow(tf, 4)     ; C_mat(3,5) = std::pow(tf, 5);
    C_mat(4,1) = 1.0; C_mat(4,2) = 2.0*tf         ; C_mat(4,3) = 3.0*std::pow(tf, 2); C_mat(4,4) = 4.0*std::pow(tf, 3) ; C_mat(4,5) = 5.0*std::pow(tf, 4);
    C_mat(5,2) = 2.0            ; C_mat(5,3) = 6.0*tf             ; C_mat(5,4) = 12.0*std::pow(tf, 2); C_mat(5,5) = 20.0*std::pow(tf, 3);

    // Solve for the coefficients
    C_mat_inv = C_mat.inverse();
    a_coeffs = C_mat_inv*bound_cond;

}

void MinJerk_OneDimension::getPos(const double & time, double & pos){
    double t;
    if (time <= to){
        t = to;
    }else if (time >= tf){
        t = tf;
    }else{
        t = time;
    }
    pos = a_coeffs[0] + a_coeffs[1]*t + a_coeffs[2]*std::pow(t,2) + a_coeffs[3]*std::pow(t,3) + a_coeffs[4]*std::pow(t,4) + a_coeffs[5]*std::pow(t, 5);
}
void MinJerk_OneDimension::getVel(const double & time, double & vel){
    double t;
    if (time <= to){
        t = to;
    }else if (time >= tf){
        t = tf;
    }else{
        t = time;
    }
    vel = a_coeffs[1] + 2.0*a_coeffs[2]*t + 3.0*a_coeffs[3]*std::pow(t,2) + 4.0*a_coeffs[4]*std::pow(t,3) + 5.0*a_coeffs[5]*std::pow(t,4);
}
void MinJerk_OneDimension::getAcc(const double & time, double & acc){
    double t;
    if (time <= to){
        t = to;
    }else if (time >= tf){
        t = tf;
    }else{
        t = time;
    }
    acc = 2.0*a_coeffs[2] + 6.0*a_coeffs[3]*t + 12.0*a_coeffs[4]*std::pow(t,2) + 20.0*a_coeffs[5]*std::pow(t,3);
}



void MinJerk_OneDimension::printParameters(){
    //printf("start and end time: (%0.4f, %0.4f)\n", to, tf);
    //myUtils::pretty_print(init_cond, std::cout, "init_cond");
    //myUtils::pretty_print(end_cond, std::cout,  "end_cond");
    //myUtils::pretty_print(C_mat, std::cout, "Coeff_Matrix");
    //myUtils::pretty_print(C_mat_inv, std::cout, "C_mat_inv");

    //myUtils::pretty_print(a_coeffs, std::cout, "Coeff_Vector");
    //myUtils::pretty_print(bound_cond, std::cout, "Boundary Conditions");
}





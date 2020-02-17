#include <Utils/Math/cubicfit_one_dim.hpp>

// Empty Constructor
CubicFit_OneDimension::CubicFit_OneDimension(){
    Initialization();
}
// Parameter Constructor
CubicFit_OneDimension::CubicFit_OneDimension(const Eigen::Vector2d & init, const Eigen::Vector2d & end,
        const double & time_start, const double & time_end){
    Initialization();
    setParams(init, end, time_start, time_end);
}

//Destructor
CubicFit_OneDimension::~CubicFit_OneDimension(){}

void CubicFit_OneDimension::Initialization(){
    // Initialize to the corrrect sizes
    C_mat = Eigen::MatrixXd::Zero(4,4);
    C_mat_inv = Eigen::MatrixXd::Zero(4,4);
    a_coeffs = Eigen::VectorXd::Zero(4);
    bound_cond = Eigen::VectorXd::Zero(4);

    init_cond = Eigen::VectorXd::Zero(2);
    end_cond =  Eigen::VectorXd::Zero(2);
    to = 0.0;
    tf = 1.0;
}

void CubicFit_OneDimension::setParams(const Eigen::Vector2d & init, const Eigen::Vector2d & end,
        const double & time_start, const double & time_end){
    // Set the Parameters
    init_cond = init;
    end_cond = end;
    bound_cond.head(2) = init_cond;
    bound_cond.tail(2) = end_cond;
    to = time_start;
    tf = time_end;

    // Construct C matrix
    C_mat(0,0) = 1.0; C_mat(0,1) = to;     C_mat(0,2) = std::pow(to, 2);     C_mat(0,3) = std::pow(to, 3);
                      C_mat(1,1) = 1.0;    C_mat(1,2) = 2.0*to;              C_mat(1,3) = 3.0*std::pow(to, 2); 
    C_mat(2,0) = 1.0; C_mat(2,1) = tf;     C_mat(2,2) = std::pow(tf, 2);     C_mat(2,3) = std::pow(tf, 3);
                      C_mat(3,1) = 1.0;    C_mat(3,2) = 2.0*tf;              C_mat(3,3) = 3.0*std::pow(tf, 2); 

    // Solve for the coefficients
    C_mat_inv = C_mat.inverse();
    a_coeffs = C_mat_inv*bound_cond;

}

void CubicFit_OneDimension::getPos(const double & time, double & pos){
    double t;
    if (time <= to){
        t = to;
    }else if (time >= tf){
        t = tf;
    }else{
        t = time;
    }
    pos = a_coeffs[0] + a_coeffs[1]*t + a_coeffs[2]*std::pow(t,2) + a_coeffs[3]*std::pow(t,3);
}
void CubicFit_OneDimension::getVel(const double & time, double & vel){
    double t;
    if (time <= to){
        t = to;
    }else if (time >= tf){
        t = tf;
    }else{
        t = time;
    }
    vel = a_coeffs[1] + 2.0*a_coeffs[2]*t + 3.0*a_coeffs[3]*std::pow(t,2);
}
void CubicFit_OneDimension::getAcc(const double & time, double & acc){
    double t;
    if (time <= to){
        t = to;
    }else if (time >= tf){
        t = tf;
    }else{
        t = time;
    }
    acc = 2.0*a_coeffs[2] + 6.0*a_coeffs[3]*t;
}

void CubicFit_OneDimension::printParameters(){
    //printf("start and end time: (%0.4f, %0.4f)\n", to, tf);
    //myUtils::pretty_print(init_cond, std::cout, "init_cond");
    //myUtils::pretty_print(end_cond, std::cout,  "end_cond");
    //myUtils::pretty_print(C_mat, std::cout, "Coeff_Matrix");
    //myUtils::pretty_print(C_mat_inv, std::cout, "C_mat_inv");

    //myUtils::pretty_print(a_coeffs, std::cout, "Coeff_Vector");
    //myUtils::pretty_print(bound_cond, std::cout, "Boundary Conditions");
}





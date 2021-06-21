#include <Utils/Math/LinearfitOneDim.hpp>

// Empty Constructor
LinearFit_OneDimension::LinearFit_OneDimension(){
    Initialization();
}
// Parameter Constructor
LinearFit_OneDimension::LinearFit_OneDimension(const double init, const double end,
                const double time_start, const double time_end){
    Initialization();
    setParams(init, end, time_start, time_end);
}

//Destructor
LinearFit_OneDimension::~LinearFit_OneDimension(){}

void LinearFit_OneDimension::Initialization(){
    // Initialize to the corrrect sizes
    C_mat = Eigen::MatrixXd::Zero(2,2);
    C_mat_inv = Eigen::MatrixXd::Zero(2,2);
    a_coeffs = Eigen::VectorXd::Zero(2);
    bound_cond = Eigen::VectorXd::Zero(2);

    init_cond = 0;
    end_cond =  0;
    to = 0.0;
    tf = 1.0;
}

void LinearFit_OneDimension::setParams(const double init, const double end,
        const double time_start, const double time_end){
    // Set the Parameters
    init_cond = init;
    end_cond = end;
    bound_cond[0] = init_cond;
    bound_cond[1] = end_cond;
    to = time_start;
    tf = time_end;

    // Construct C matrix
    C_mat(0,0) = 1.0; C_mat(0,1) = to;
    C_mat(1,0) = 1.0; C_mat(1,1) = tf;

    // Solve for the coefficients
    C_mat_inv = C_mat.inverse();
    a_coeffs = C_mat_inv*bound_cond;
}

void LinearFit_OneDimension::getPos(const double time, double & pos){
    double t;
    if (time <= to){
        t = to;
    }else if (time >= tf){
        t = tf;
    }else{
        t = time;
    }
    pos = a_coeffs[0] + a_coeffs[1]*t;
}
void LinearFit_OneDimension::getVel(const double time, double & vel){
    vel = a_coeffs[1];
}
void LinearFit_OneDimension::getAcc(const double time, double & acc){
    acc = 0.0;
}

void LinearFit_OneDimension::printParameters(){
    //printf("start and end time: (%0.4f, %0.4f)\n", to, tf);
    //myUtils::pretty_print(init_cond, std::cout, "init_cond");
    //myUtils::pretty_print(end_cond, std::cout,  "end_cond");
    //myUtils::pretty_print(C_mat, std::cout, "Coeff_Matrix");
    //myUtils::pretty_print(C_mat_inv, std::cout, "C_mat_inv");

    //myUtils::pretty_print(a_coeffs, std::cout, "Coeff_Vector");
    //myUtils::pretty_print(bound_cond, std::cout, "Boundary Conditions");
}





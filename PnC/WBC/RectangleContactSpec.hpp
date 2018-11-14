#pragma once

#include <PnC/WBC/ContactSpec.hpp>

class RobotSystem;

class RectangleContactSpec : public ContactSpec
{
private:
    std::string linkName_;

    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();

public:
    RectangleContactSpec(RobotSystem * _robot,
                          const std::string & _link_name,
                          const double & _mu );
    virtual ~RectangleContactSpec() {};

    void setMaxFz(double max_fz) { max_Fz_ = max_fz; }


    std::string link_name_;
    Eigen::Vector3d vec_bodynode_to_contact_surface;
    double mu_;
    double max_Fz_;
};

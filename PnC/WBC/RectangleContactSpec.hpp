#pragma once

#include <PnC/WBC/ContactSpec.hpp>

class RobotSystem;

class RectangleContactSpec : public ContactSpec
{
private:
    RobotSystem* robot_;
    std::string linkName_;

public:
    RectangleContactSpec(RobotSystem * _robot,
                          const std::string & _link_name,
                          const double & _mu );
    virtual ~RectangleContactSpec() {};

    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();

    std::string link_name_;
    Eigen::Vector3d vec_bodynode_to_contact_surface;
    double mu_;
};

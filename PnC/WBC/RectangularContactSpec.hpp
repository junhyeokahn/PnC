#pragma once

#include <PnC/WBC/ContactSpec.hpp>

class RobotSystem;

class RectangularContactSpec : public ContactSpec
{
private:
    std::string linkName_;

    virtual bool _UpdateContactGeometry();
    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();

public:
    RectangularContactSpec(RobotSystem * _robot,
                          const std::string & _link_name,
                          const double & _mu );
    virtual ~RectangularContactSpec() {};

    void setMaxFz(double max_fz) { max_Fz_ = max_fz; }


    std::string link_name_;
    Eigen::Isometry3d iso_world_to_contact_center_;
    Eigen::Vector3d vec_bodynode_to_contact_surface_;
    Eigen::Vector3d box_size_;
    double mu_;
    double max_Fz_;
};

#pragma once

#include <PnC/WBC/ContactSpec.hpp>

class RobotSystem;

class PointContact: public ContactSpec{
    public:
        PointContact(RobotSystem* robot, const std::string& _link_name, const double & _mu);
        virtual ~PointContact();

        void setMaxFz(double max_fz){ max_Fz_ = max_fz; }

    protected:
        double mu_;
        double max_Fz_;
        std::string link_name_;

        virtual bool _UpdateJc();
        virtual bool _UpdateJcDotQdot();
        virtual bool _UpdateUf();
        virtual bool _UpdateInequalityVector();
};

#pragma once

#include <PnC/WBC/ContactSpec.hpp>

class RobotSystem;

class LineContact: public ContactSpec{
    public:
        LineContact(RobotSystem* robot, const std::string& _link_name, const double & _mu, const double & _gamma);
        virtual ~LineContact();

        void setMaxFz(double max_fz){ max_Fz_ = max_fz; }

    protected:
        double mu_;
        double gamma_;
        double max_Fz_;
        std::string link_name_;

        virtual bool _UpdateContactGeometry();
        virtual bool _UpdateJc();
        virtual bool _UpdateJcDotQdot();
        virtual bool _UpdateUf();
        virtual bool _UpdateInequalityVector();
};

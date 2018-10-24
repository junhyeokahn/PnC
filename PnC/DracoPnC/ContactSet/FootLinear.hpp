#pragma once

#include <PnC/WBC/ContactSpec.hpp>

class RobotSystem;

class FootLinear: public ContactSpec{
    public:
        FootLinear(RobotSystem* robot, const std::string& _link_name, const double & _mu);
        virtual ~FootLinear();

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

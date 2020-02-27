#include <PnC/DracoPnC/DracoInterface.hpp>
#include <ADE/DracoSim.hpp>

class DracoWrapper {
private:
    //PnC, sim, and robot members
    DracoInterface* interface_;
    DracoSim* simulator_;

    //Logical Safety
    bool running_;

public:
    DracoWrapper();

    //Set up the DracoWrapper to create the PnC instance and pass it down into the sim and ros nodelet to centralize commands
    void Initialize();
    //Draco Walking Methods
    void SetWalkRawCommand(double ft_length, double r_ft_width, double l_ft_width,
                        double ori_inc, int num_step);
    void SetWalkXCommand(double x);
    void SetWalkYCommand(double y);
    void SetTurnCommand(double th);
    void SetWalkToRelativeCommand(double x, double y, double th);
    void SetHaltCommand();
    //Scorio Manipulation Methods
    void SetMoveEndEffectorCommand(double x, double y, double z, double qw, double qx, double qy, double qz);
    void SetCloseGripperCommand();
    void SetOpenGripperCommand();
    //Shutdown the sim and ros nodelet and destruct the PnC instance
    void Shutdown();
};



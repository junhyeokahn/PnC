#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Eigen/Dense>

class Interface;
class DracoHipSensorData;
class DracoHipCommand;

class DracoHipWorldNode : public dart::gui::osg::WorldNode
{
private:
    Interface* mInterface;
    DracoHipSensorData* mSensorData;
    DracoHipCommand* mCommand;

    dart::dynamics::SkeletonPtr mSkel;
    Eigen::VectorXd mTorqueCommand;
    int mDof;

    Eigen::VectorXd mInit;
    bool mIsVisualizeTrajectory;
    std::string mTrajectoryFile;
    std::vector<std::string> mFile;
    std::string* mLine;
    Eigen::VectorXd mPos;
    Eigen::VectorXd mVel;

public:
    DracoHipWorldNode(const dart::simulation::WorldPtr & world,
                      osgShadow::MinimalShadowMap *);
    virtual ~DracoHipWorldNode();

    void customPreStep() override;
};

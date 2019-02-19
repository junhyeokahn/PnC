#pragma once

#include <string>
#include <vector>

#include <dart/dart.hpp>

#include <PnC/DracoPnC/DracoMoCapManager.hpp>
#include <Utils/IO/Pthread.hpp>

class DracoLedPosAnnouncer : public Pthread {
   public:
    DracoLedPosAnnouncer(dart::simulation::WorldPtr _world);
    virtual ~DracoLedPosAnnouncer(void) {}

    virtual void run(void);

   protected:
    int count_;
    int socket_;
    std::vector<std::string> led_link_name_list_;

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr skel_;
    dart::dynamics::SkeletonPtr ground_;
};

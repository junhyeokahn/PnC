#pragma once

#include <string>
#include <vector>

#include <dart/dart.hpp>

#include <PnC/DracoPnC/DracoMoCapManager.hpp>
#include <Utils/IO/Pthread.hpp>

class DracoLedPosAnnouncer : public Pthread {
   public:
    DracoLedPosAnnouncer();
    virtual ~DracoLedPosAnnouncer(void) {}

    virtual void run(void);

    std::vector<int> led_link_idx_list;
    draco_message msg;

   protected:
    void _Print_message();

    int count_;
    int socket_;
};

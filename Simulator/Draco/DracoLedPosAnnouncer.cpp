#include "DracoLedPosAnnouncer.hpp"
#include <Addition/DataManager/data_protocol.h>
#include <Utils/IO/comm_udp.hpp>

DracoLedPosAnnouncer::DracoLedPosAnnouncer(dart::simulation::WorldPtr _world)
    : socket_(0), count_(0) {
    world_ = _world;
    skel_ = world_->getSkeleton("Draco");
    ground_ = world_->getSkeleton("ground_skeleton");
}

void DracoLedPosAnnouncer::run() {
    // printf("[LED Position Announcer] Start \n");

    draco_message draco_message;
    led_link_name_list_.clear();

    // Link name list !! must match with MocapLed enum !!
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("lTorsoLed");
    led_link_name_list_.push_back("rTorsoLed");

    // Null link
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("cTorsoLed");
    led_link_name_list_.push_back("cTorsoLed");

    while (true) {
        for (int j(0); j < NUM_MARKERS; ++j) {
            draco_message.visible[j] = 1;
            for (int i(0); i < 3; ++i) {
                draco_message.data[3 * j + i] =
                    skel_->getBodyNode(led_link_name_list_[j])
                        ->getTransform()
                        .translation()[i] *
                    1000.0;
            }
        }

        COMM::send_data(socket_, MOCAP_DATA_PORT, &draco_message,
                        sizeof(draco_message), IP_ADDR_MYSELF);
        usleep(2000);
        ++count_;
        // if(count_%100 == 1)  printf("count: %d\n", count_);
    }
}


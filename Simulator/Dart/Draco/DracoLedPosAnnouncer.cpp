#include <Simulator/Dart/Draco/DracoLedPosAnnouncer.hpp>
#include <Addition/DataManager/data_protocol.h>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/IO/comm_udp.hpp>

DracoLedPosAnnouncer::DracoLedPosAnnouncer() : socket_(0), count_(0) {
    // Link name list !! must match with MocapLed enum !!
    led_link_idx_list.resize(NUM_MARKERS, 0);
    led_link_idx_list[0] = DracoBodyNode::cTorsoLed;
    led_link_idx_list[1] = DracoBodyNode::lTorsoLed;
    led_link_idx_list[2] = DracoBodyNode::rTorsoLed;
    for (int led_idx = 0; led_idx < NUM_MARKERS; ++led_idx) {
        msg.visible[led_idx] = 0;
        for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
            msg.data[3 * led_idx + axis_idx] = 0.;
        }
    }
}

void DracoLedPosAnnouncer::run() {
    // printf("[LED Position Announcer] Start \n");

    while (true) {
        COMM::send_data(socket_, MOCAP_DATA_PORT, &msg, sizeof(msg),
                        IP_ADDR_MYSELF);
        usleep(2000);
        ++count_;
        // if (count_ % 500 == 1) {
        // printf("sender count: %d\n", count_);
        //_Print_message();
        //}
    }
}

void DracoLedPosAnnouncer::_Print_message() {
    for (int i(0); i < NUM_MARKERS; ++i) {
        printf("%d th LED data (cond, x, y, z): %d, (%f, %f, %f) \n", i,
               msg.visible[i], msg.data[3 * i], msg.data[3 * i + 1],
               msg.data[3 * i + 2]);
        if (i == (NUM_MARKERS - 1)) {
            // printf("size: %lu\n", sizeof(draco_message));
            printf("\n");
        }
    }
}

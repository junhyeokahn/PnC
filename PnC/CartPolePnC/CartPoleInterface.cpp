#include <stdio.h>
#include <chrono>
#include <thread>

#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <Configuration.h>

CartPoleInterface::CartPoleInterface() : Interface() {
    // Construct zmq publisher
    context_ = new zmq::context_t(1);
    publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
    publisher_->bind(RL_LOCAL_HOST);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // build neural net policy
}

CartPoleInterface::~CartPoleInterface() {
    delete context_;
    delete publisher_;
}

void CartPoleInterface::getCommand( void* _data, void* _cmd ) {

    CartPoleCommand* cmd = ((CartPoleCommand*) _cmd);
    CartPoleSensorData* data = ((CartPoleSensorData*) _data);

    cmd->jtrq = 0.;
}

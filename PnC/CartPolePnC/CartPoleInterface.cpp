#include <stdio.h>

#include <PnC/CartPolePnC/CartPoleInterface.hpp>

CartPoleInterface::CartPoleInterface() : Interface() {
    // build neural net policy
}

CartPoleInterface::~CartPoleInterface() {}

void CartPoleInterface::getCommand( void* _data, void* _cmd ) {

    CartPoleCommand* cmd = ((CartPoleCommand*) _cmd);
    CartPoleSensorData* data = ((CartPoleSensorData*) _data);

    cmd->jtrq = 0.;
}

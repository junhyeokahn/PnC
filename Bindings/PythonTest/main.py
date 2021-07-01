import sys
import os
sys.path.append('/home/knapsack/project/locomotion/src/PnC/build/lib')

import A1Interface
import numpy as np

def main():
    interface_ = A1Interface.A1Interface()
    sensor_data_ = A1Interface.A1SensorData()
    command_ = A1Interface.A1Command()

    #######################
    # Set up the Simulation
    #######################

    while True:
        #################################
        # Set Sensor Data from Simulation
        #################################
        sensor_data_.imu_ang_vel = np.zeros(3)
        sensor_data_.imu_acc = np.zeros(3)

        sensor_data_.q = np.zeros(12)
        sensor_data_.qdot = np.zeros(12)
        sensor_data_.jtrq = np.zeros(12)
        sensor_data_.virtual_q = np.zeros(6)
        sensor_data_.virtual_qdot = np.zeros(6)

        sensor_data_.flfoot_contact = True
        sensor_data_.frfoot_contact = True
        sensor_data_.rlfoot_contact = True
        sensor_data_.rrfoot_contact = True

        ##############################
        # Call Interface to getCommand
        ##############################
        interface_.getCommand(sensor_data_, command_)

        #####################################
        # Compute PD Values and add to Torque
        #####################################
        trq_cmd_ = np.zeros(18)
        pos_cmd_ = np.zeros(12)
        vel_cmd_ = np.zeros(12)
        pos_cmd_ = command_.q
        vel_cmd_ = command_.qdot

        for i in range(12):
            trq_cmd_[i+6]= kp_[i] * (pos_cmd_[i] - sensor_data_.q[i]) +
                           kd_[i] * (vel_cmd_[i] - sensor_data_.qdot[i]) +
                           command_.jtrq[i]

        ########################
        # Feed data to simulator
        ########################

    # end While True

    print("Done")


if __name__ == "__main__":
    main()

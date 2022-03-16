import sys
import os
sys.path.append('/home/knapsack/project/locomotion/src/PnC/build/lib')

import A1Interface
import ConvexMPC
import numpy as np

def main():
  interface_ = A1Interface.A1Interface()
  sensor_data_ = A1Interface.A1SensorData()
  command_ = A1Interface.A1Command()

  # MPC Constructor Inputs
  mass = float(11.7)
  num_legs = int(4)
  body_inertia = (0.07335, 0.0, 0.0, 0.0, 0.25068, 0.0, 0.0, 0.0, 0.25447)
  _MPC_WEIGHTS = (5., 5., 0.2, 0., 0., 50, 0.5, 0.5, 0.2, 10., 5., 0.1, 0.)
  _PLANNING_HORIZON_STEPS = int(10)
  _PLANNING_TIMESTEP = float(0.025)
  qp_solver = ConvexMPC.OSQP

  mpc_ = ConvexMPC.ConvexMPC(mass, body_inertia, num_legs,
                             _PLANNING_HORIZON_STEPS,
                             _PLANNING_TIMESTEP,
                             _MPC_WEIGHTS, 1e-5, 
                             qp_solver)

  #######################
  # Set up the Simulation
  #######################

  print("Initialized")

  # while True:
  #     #################################
  #     # Set Sensor Data from Simulation
  #     #################################
  #     sensor_data_.imu_ang_vel = np.zeros(3)
  #     sensor_data_.imu_acc = np.zeros(3)

  #     sensor_data_.q = np.zeros(12)
  #     sensor_data_.qdot = np.zeros(12)
  #     sensor_data_.jtrq = np.zeros(12)
  #     sensor_data_.virtual_q = np.zeros(6)
  #     sensor_data_.virtual_qdot = np.zeros(6)

  #     sensor_data_.flfoot_contact = True
  #     sensor_data_.frfoot_contact = True
  #     sensor_data_.rlfoot_contact = True
  #     sensor_data_.rrfoot_contact = True

  #     ##############################
  #     # Call Interface to getCommand
  #     ##############################
  #     interface_.getCommand(sensor_data_, command_)

  #     #####################################
  #     # Compute PD Values and add to Torque
  #     #####################################
  #     trq_cmd_ = np.zeros(18)
  #     pos_cmd_ = np.zeros(12)
  #     vel_cmd_ = np.zeros(12)
  #     pos_cmd_ = command_.q
  #     vel_cmd_ = command_.qdot

  #     kp_ = np.zeros(12)
  #     kd_ = np.zeros(12)

  #     for i in range(12):
  #         trq_cmd_[i+6]= kp_[i] * (pos_cmd_[i] - sensor_data_.q[i]) + \
  #                        kd_[i] * (vel_cmd_[i] - sensor_data_.qdot[i]) + \
  #                        command_.jtrq[i]

      ########################
      # Feed data to simulator
      ########################

  # end While True

  print("Done")


if __name__ == "__main__":
  main()

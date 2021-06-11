import sys
import os
sys.path.append(os.getcwd()+'/build/lib')

import A1Interface
import numpy as np

def main():
    draco_interface = A1Interface.A1Interface()
    draco_sensor_data = A1Interface.A1SensorData()
    draco_command = A1Interface.A1Command()

    # draco_sensor_data.imu_ang_vel = np.zeros(3)
    # draco_sensor_data.imu_acc = np.array([0, 0, -9.81])
    # draco_sensor_data.q = np.array([0, 0, -0.5, 1.8, 1.03, 0, 0, -0.5, 1.8, 1.03])
    # draco_sensor_data.qdot = np.zeros(10)
    # draco_sensor_data.jtrq = np.zeros(10)
    # draco_sensor_data.temperature = np.zeros(10)
    # draco_sensor_data.motor_current = np.zeros(10)
    # draco_sensor_data.bus_voltage = np.zeros(10)
    # draco_sensor_data.bus_current = np.zeros(10)
    # draco_sensor_data.rotor_inertia = np.zeros(10)
    # draco_sensor_data.rfoot_ati = np.zeros(6)
    # draco_sensor_data.lfoot_ati = np.zeros(6)
    # draco_sensor_data.rfoot_contact = True
    # draco_sensor_data.lfoot_contact = True
    # draco_command.turn_off = False
    # draco_command.q = np.zeros(10)
    # draco_command.qdot = np.zeros(10)
    # draco_command.jtrq = np.zeros(10)

    #for i in range(10):
        #draco_interface.getCommand(draco_sensor_data, draco_command)

    print("Done")


if __name__ == "__main__":
    main()

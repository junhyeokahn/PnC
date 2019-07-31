import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1


def create_figures(subfigure_width=480, subfigure_height=500, starting_figure_no=1, \
        starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

    ## read files  ************************************************************
    # CoM data
    data_global_pos_offset = \
        np.genfromtxt(file_path+'global_pos_local.txt', delimiter=None, dtype=(float))
    data_com_pos = \
        np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float)) ##
    data_com_vel = \
        np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float)) ##
    data_config = \
        np.genfromtxt(file_path+'config.txt', delimiter=None, dtype=(float))
    data_config_dot = \
        np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))
    # foot data #
    data_rfoot_contact = \
        np.genfromtxt(file_path+'rfoot_contact.txt', delimiter=None, dtype=(float))
    data_rfoot_pos = \
        np.genfromtxt(file_path+'rfoot_pos.txt', delimiter=None, dtype=(float))
    data_rfoot_pos_des = \
        np.genfromtxt(file_path+'rfoot_pos_des.txt', delimiter=None, dtype=(float))
    data_lfoot_contact = \
        np.genfromtxt(file_path+'lfoot_contact.txt', delimiter=None, dtype=(float))
    data_lfoot_pos = \
        np.genfromtxt(file_path+'lfoot_pos.txt', delimiter=None, dtype=(float))
    data_lfoot_pos_des = \
        np.genfromtxt(file_path+'lfoot_pos_des.txt', delimiter=None, dtype=(float))
    data_phase = \
        np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    data_planner = \
        np.genfromtxt(file_path+'planner_data.txt', delimiter=None, dtype=(float))
    data_est_mocap_body_vel = \
            np.genfromtxt(file_path+'est_mocap_body_vel.txt', delimiter=None, dtype=(float))
    data_est_com_vel = \
            np.genfromtxt(file_path+'est_com_vel.txt', delimiter=None, dtype=(float)) ##

    phase_swing_st_trans_idx = []
    phase_swing_st_idx = []
    phase_swing_end_trans_idx = []
    phase_double_idx = []

    for i in range(len(data_phase)-1):
            if data_phase[i] != data_phase[i+1] and (data_phase[i+1]== 3 or data_phase[i+1] == 7):
                phase_swing_st_trans_idx.append(i+1)
            elif data_phase[i] != data_phase[i+1] and (data_phase[i+1]== 4 or data_phase[i+1] == 8):
                phase_swing_st_idx.append(i+1)
            elif data_phase[i] != data_phase[i+1] and (data_phase[i+1]== 5 or data_phase[i+1] == 9):
                phase_swing_end_trans_idx.append(i+1)
            elif data_phase[i] != data_phase[i+1] and (data_phase[i+1]== 6 or data_phase[i+1] == 2):
                phase_double_idx.append(i+1)
            else:
                pass


    stance_foot_loc = []
    for i in range(len(data_global_pos_offset)-1):
        if (data_global_pos_offset[i,1] != data_global_pos_offset[i+1,1]):
            # stance_foot_loc.append(data_global_pos_offset[i+1,:])
            stance_foot_loc.append(data_global_pos_offset[i,:]) #jh

    np_stancefoot = np.array(stance_foot_loc)

    data_com_pos_global = data_com_pos + data_global_pos_offset
    data_body_global = data_config[:,0:3] + data_global_pos_offset


    # st_idx=35
    # end_idx=st_idx+3
    st_idx=34
    end_idx=st_idx+3
    # st_idx=18
    # end_idx=st_idx+3
    f, a = plt.subplots()
    a.plot(data_planner[st_idx:end_idx, 0], data_planner[st_idx:end_idx,2], '.')
    # a.plot(data_planner[st_idx:end_idx, 8], [0]*(end_idx-st_idx), '*', color='red', markersize=12)
    # a.plot(np_stancefoot[st_idx:end_idx,0], [0]*(end_idx-st_idx), '*', color='orange', markersize=12)
    # a.plot(np_stancefoot[end_idx,0], 0, '*', markersize=12)
    f.suptitle("x-xdot")
    a.grid()
    # plt.savefig('Users/junhyeokahn/Repository/PnC/ExperimentData/x-xdot2.eps', format='eps', dpi=1000)
    plt.savefig('x-xdot2.eps', format='eps', dpi=1000)


if __name__ == "__main__":
   create_figures()
   plt.show()

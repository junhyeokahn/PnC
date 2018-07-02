import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 2

# number of state
num_dim = 3

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../ExperimentDataCheck/"

    ## read files
    data_time = np.genfromtxt(file_path+'Time.txt', delimiter='\n', dtype=(float))
    data_cent_pos_des= \
    np.genfromtxt(file_path+'CentroidTaskPosDes.txt', delimiter=None, dtype=(float))
    data_cent_pos_act= \
    np.genfromtxt(file_path+'CentroidTaskPosAct.txt', delimiter=None, dtype=(float))
    data_cent_vel_des= \
    np.genfromtxt(file_path+'CentroidTaskVelDes.txt', delimiter=None, dtype=(float))
    data_cent_vel_act= \
    np.genfromtxt(file_path+'CentroidTaskVelAct.txt', delimiter=None, dtype=(float))

    axes = plt.gca()

    ## plot CoM Pos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('CoM Position')
    for i in range(1, num_dim+1, 1):
        ax1 = plt.subplot(num_dim, 1, i)
        plt.plot(data_time, data_cent_pos_des[:, 3+i-1], "r-")
        plt.plot(data_time, data_cent_pos_act[:, 3+i-1], "b-")
        plt.grid(True)
    plt.xlabel('time (sec)')

    ## plot CoM Vel
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('CoM Velocity')
    for i in range(1, num_dim+1, 1):
        ax1 = plt.subplot(num_dim, 1, i)
        plt.plot(data_time, data_cent_vel_des[:, 3+i-1], "r-")
        plt.plot(data_time, data_cent_vel_act[:, 3+i-1], "b-")
        plt.grid(True)
    plt.xlabel('time (sec)')

if __name__ == "__main__":
    create_figures()
    plt.show()

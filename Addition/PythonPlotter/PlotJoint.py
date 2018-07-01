import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of state
task_dim = 10

# number of figures in this plot
num_figures = task_dim

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../ExperimentDataCheck/"

    ## read files
    data_time = np.genfromtxt(file_path+'Time.txt', delimiter='\n', dtype=(float))
    data_jpos_des= \
    np.genfromtxt(file_path+'JointTaskPosDes.txt', delimiter=None, dtype=(float))
    data_jvel_des= \
    np.genfromtxt(file_path+'JointTaskVelDes.txt', delimiter=None, dtype=(float))
    data_jpos_act= \
    np.genfromtxt(file_path+'JointTaskPosAct.txt', delimiter=None, dtype=(float))
    data_jvel_act= \
    np.genfromtxt(file_path+'JointTaskVelAct.txt', delimiter=None, dtype=(float))

    # __import__('ipdb').set_trace()
    # exit()
    axes = plt.gca()

    for i in range(num_figures):
        fig = plt.figure(figure_number)
        plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
        fig.canvas.set_window_title('Q')
        ax1 = plt.subplot(2, 1, 1)
        plt.plot(data_time, data_jpos_des[:, i], "r-")
        plt.plot(data_time, data_jpos_act[:, i], "b-")
        plt.grid(True)
        ax1 = plt.subplot(2, 1, 2)
        plt.plot(data_time, data_jvel_des[:, i], "r-")
        plt.plot(data_time, data_jvel_act[:, i], "r-")
        plt.xlabel('time (sec)')
        figure_number += 1
        if plot_configuration == PLOT_HORIZONTALLY:
            col_index += 1
        elif plot_configuration == PLOT_VERTICALLY:
            row_index +=1

if __name__ == "__main__":
    create_figures()
    plt.show()

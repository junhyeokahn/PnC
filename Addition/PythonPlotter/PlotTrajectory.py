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

# number of state, input
num_state = "global"
num_input = "global"
file_path = "global"

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    ## read files
    data_time_state_input = \
    np.genfromtxt(file_path, delimiter=None, dtype=(float))

    axes = plt.gca()

    ## plot state
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('State')
    for i in range(1, num_state+1, 1):
        ax1 = plt.subplot(num_state, 1, i)
        plt.plot(data_time_state_input[:, 0], data_time_state_input[:, i], "r-")
        plt.grid(True)
    plt.xlabel('time (sec)')

    ## plot input
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('Input')
    for i in range(1, num_input+1, 1):
        ax1 = plt.subplot(num_input, 1, i)
        plt.plot(data_time_state_input[:, 0], data_time_state_input[:, num_state+i], "r-")
        plt.grid(True)
    plt.xlabel('time (sec)')


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", type=str, default="/CartPolePnC/OfflineTrajectoryGeneration/TrajectoriesBin/dircol_swing_up_1_4_1")
    args = parser.parse_args()
    num_state = int((args.file).split("_")[-2])
    num_input = int((args.file).split("_")[-1])
    file_path = os.getcwd() + "/../../PnC" + args.file + ".txt"

    create_figures()
    plt.show()

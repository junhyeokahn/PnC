import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os
import ipdb

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1


def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, \
        starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):

    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    ## read files --------------------------------------------------------------------
    file_path = os.getcwd() + "/../../../ExperimentData/"

    data_q = \
            np.genfromtxt(file_path+'q.txt', delimiter=None, dtype=(float))
    data_qdes = \
            np.genfromtxt(file_path+'q_des.txt', delimiter=None, dtype=(float))
    data_qdot = \
            np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))
    data_qdotdes = \
            np.genfromtxt(file_path+'qdot_des.txt', delimiter=None, dtype=(float))
    data_cmd = \
            np.genfromtxt(file_path+'gamma.txt', delimiter=None, dtype=(float))

    f, ax = plt.subplots()
    ax.plot(data_cmd[:, 0])
    ax.grid()
    f.suptitle("trq cmd")


    f, ax = plt.subplots()
    ax.plot(data_q[:, 0], color='b')
    ax.plot(data_qdes[:, 0], color='r')
    ax.grid()
    f.suptitle("q")


    f, ax = plt.subplots()
    ax.plot(data_qdot[:, 0], color='b')
    ax.plot(data_qdotdes[:, 0], color='r')
    ax.grid()
    f.suptitle("qdot")




if __name__ == "__main__":
    create_figures()
    plt.show()


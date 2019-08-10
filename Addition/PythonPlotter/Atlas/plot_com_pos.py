import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1


def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, \
        starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):

    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    ## read files --------------------------------------------------------------------
    file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

    data_com_pos_des = \
            np.genfromtxt(file_path+'com_pos_des.txt', delimiter=None, dtype=(float))
    data_com_pos = \
            np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))
    data_com_vel_des = \
            np.genfromtxt(file_path+'com_vel_des.txt', delimiter=None, dtype=(float))
    data_com_vel = \
            np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))
    data_com_acc_des = \
            np.genfromtxt(file_path+'com_acc_des.txt', delimiter=None, dtype=(float))
    data_jacc_des_cmd = \
            np.genfromtxt(file_path+'jacc_des.txt', delimiter=None, dtype=(float))
    data_trq_cmd = \
            np.genfromtxt(file_path+'command.txt', delimiter=None, dtype=(float))

    data_x = np.genfromtxt(file_path+'running_time.txt', delimiter='\n', dtype=(float))
    num_dim = 3
    num_adof = 23 
    
    st_idx = 1;
    end_idx = len(data_x) - 1
    data_x = data_x[st_idx:end_idx]
    ##--------------------------------------------------------------------------------
    # PHASE MARKER #
    data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    # get phase.txt data #
    phseChange = []
    for i in range(0,len(data_x)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i - st_idx)
        else:
            pass

    # Plot Figure --------------------------------------------------------------------
    ## plot com_pos_des/com_pos 
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('com_pos')
    for i in range(1,num_dim + 1,1):
        ax1 = plt.subplot(num_dim, 1, i)
        plt.plot(data_x, data_com_pos[st_idx:end_idx,i-1], "b-")
        plt.plot(data_x, data_com_pos_des[st_idx:end_idx,i-1], "r-")

        # plt.legend(('command', 'pos'), loc='upper left')
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    #----------------------------------------------------------------------------------
    ## plot com_vel_des/com_vel 
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('com_vel')
    for i in range(1,num_dim + 1,1):
        ax1 = plt.subplot(num_dim, 1, i)
        plt.plot(data_x, data_com_vel[st_idx:end_idx,i-1], "b-")
        plt.plot(data_x, data_com_vel_des[st_idx:end_idx,i-1], "r-")

        # plt.legend(('command', 'pos'), loc='upper left')
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    #----------------------------------------------------------------------------------
    ## plot com_acc_des 
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('com_acc')
    for i in range(1,num_dim + 1,1):
        ax1 = plt.subplot(num_dim, 1, i)
        plt.plot(data_x, data_com_acc_des[st_idx:end_idx,i-1], "r-")

        # plt.legend(('command', 'pos'), loc='upper left')
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
  #----------------------------------------------------------------------------------

     ## plot jacc_cmd 
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('des_jacc_cmd')
    for i in range(1,num_adof + 1,1):
        ax1 = plt.subplot(num_adof, 1, i)
        plt.plot(data_x, data_jacc_des_cmd[st_idx:end_idx,i-1], "r-")

        # plt.legend(('command', 'pos'), loc='upper left')
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

   #----------------------------------------------------------------------------------
    ## plot trq_cmd 
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('trq_cmd')
    for i in range(1,num_adof + 1,1):
        ax1 = plt.subplot(num_adof, 1, i)
        plt.plot(data_x, data_trq_cmd[st_idx:end_idx,i-1], "r-")

        # plt.legend(('command', 'pos'), loc='upper left')
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

  #----------------------------------------------------------------------------------

if __name__ == "__main__":
    create_figures()
    plt.show()


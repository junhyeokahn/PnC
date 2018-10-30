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
    file_path = os.getcwd() + "/../../../experiment_data_check/"

    data_motor_current = \
            np.genfromtxt(file_path+'motor_current.txt', delimiter=None, dtype=(float))
    data_temperature = \
           np.genfromtxt(file_path+'temperature.txt', delimiter=None, dtype=(float))
    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))
    num_leg_joint = 5

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
    ## plot command/jpos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('motor current (left leg)')
    for i in range(1,num_leg_joint+1,1):
        ax1 = plt.subplot(num_leg_joint, 1, i)
        plt.plot(data_x, data_motor_current[st_idx:end_idx,i-1], "b-")

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

    # Plot Figure --------------------------------------------------------------------
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('motor current (right leg)')
    for i in range(1,num_leg_joint + 1,1):
        ax1 = plt.subplot(num_leg_joint, 1, i)
        plt.plot(data_x, data_motor_current[st_idx:end_idx,i-1 + num_leg_joint], "b-")
        
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

    # Plot Figure --------------------------------------------------------------------
    ## plot jvel
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('motor temperature (left_leg)')
    for i in range(1,num_leg_joint + 1,1):
        ax1 = plt.subplot(num_leg_joint, 1, i)
        plt.plot(data_x, data_temperature[st_idx:end_idx, i-1], "b-")

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

    # Plot Figure --------------------------------------------------------------------
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('motor temperature (right_leg)')
    for i in range(1,num_leg_joint + 1,1):
        ax1 = plt.subplot(num_leg_joint, 1, i)
        plt.plot(data_x, data_temperature[st_idx:end_idx, i-1 + num_leg_joint], "b-");
       
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


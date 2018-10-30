import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 4
num_leg_joint = 5

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, \
        starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

    ## read files
    data_global_pos_offset = \
    np.genfromtxt(file_path+'global_pos_local.txt', delimiter=None, dtype=(float))
    # data_body_des = \
    # np.genfromtxt(file_path+'body_pos_des.txt', delimiter=None, dtype=(float))
    # data_body_ori_des = \
    # np.genfromtxt(file_path+'body_ori_des.txt', delimiter=None, dtype=(float))
    # data_body_ori = \
    # np.genfromtxt(file_path+'body_ori.txt', delimiter=None, dtype=(float))
    # data_body_vel = \
    # np.genfromtxt(file_path+'body_vel.txt', delimiter=None, dtype=(float))
    data_qdot = \
    np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))
    data_q = \
    np.genfromtxt(file_path+'config.txt', delimiter=None, dtype=(float))
   # data_ekf_body_pos = \
    # np.genfromtxt(file_path+'ekf_o_r.txt', delimiter=None, dtype=(float))
    # data_ekf_body_vel = \
    # np.genfromtxt(file_path+'ekf_o_v.txt', delimiter=None, dtype=(float))
    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

    data_body_ori = np.zeros(shape = (len(data_x), 3))
    data_body_ori = np.copy(data_q[:, 3:6]);
    
    st_idx = 1
    end_idx = len(data_x) - 10
    # end_idx = st_idx + 3000
    data_x = data_x[st_idx:end_idx]

    # PHASE MARKER #
    data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    data_phse = data_phse[st_idx:end_idx]
    # get phase.txt data #
    phseChange = []
    for i in range(0,len(data_x)-1):
            if data_phse[i] != data_phse[i+1]:
                phseChange.append(i)
            else:
                pass
    axes = plt.gca()

    stand_up_idx = 1
    stand_up_idx += st_idx  

    data_body_global = data_q[:,0:3] + data_global_pos_offset

    ## plot global
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('body global pos')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_body_global[st_idx:end_idx,i-1], "b-")
        # plt.plot(data_x, data_global_pos_offset[st_idx:end_idx,i-1], "crimson")

        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    ## plot local body
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('body local pos')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_q[st_idx:end_idx,i-1], "b-")
        
        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + \
            "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('body vel')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_qdot[st_idx:end_idx,i-1], "b")

        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
        
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body ori (Euler ZYX)')

    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_body_ori[st_idx:end_idx,i-1], "b-")
        plt.grid(True)

        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body ang vel')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_qdot[st_idx:end_idx,i-1+3], "r-")
        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.xlabel('time (sec)')


if __name__ == "__main__":
    create_figures()
    plt.show()

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1


num_figures = 5
num_leg_joint = 5

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../../experiment_data_check/"
## read files
    data_cmd = \
            np.genfromtxt(file_path+'command.txt', delimiter=None, dtype=(float))
    # data_filter_cmd = \
            # np.genfromtxt(file_path+'filtered_cmd.txt', delimiter=None, dtype=(float))
    data_torque = \
            np.genfromtxt(file_path+'torque.txt', delimiter=None, dtype=(float))
    # data_motor_current = \
            # np.genfromtxt(file_path+'motor_current.txt', delimiter=None, dtype=(float))
    speed_ratio_lin = 200;
    speed_ratio_rot = 2942* 0.034;
    torque_const = 0.039;
    eff = -0.7
    scale = [speed_ratio_lin * torque_const*eff, \
            speed_ratio_rot * torque_const*eff, \
            speed_ratio_rot * torque_const*eff];
    # data_qddot_cmd = \
            # np.genfromtxt(file_path+'qddot_cmd.txt', delimiter=None, dtype=(float))
    
    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))
    
    st_idx = 1
    end_idx = len(data_x) - 1
    data_x = data_x[st_idx:end_idx]
    # PHASE MARKER #
    data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    # get phase.txt data #
    phseChange = []
    for i in range(0,len(data_x)-1):
            if data_phse[i] != data_phse[i+1]:
                phseChange.append(i - st_idx)
            else:
                pass

    # LeftFoot Contact Signal #
    data_lf_contact = np.genfromtxt(file_path+'lfoot_contact.txt', delimiter=None, dtype=(float))
    data_lf_contact = data_lf_contact[st_idx:end_idx]    
    lf_contact_index_change = []
    lf_contact_index_change.append(0)    
    lf_contact_index_change.append(1)        
    for i in range(2, len(data_x)):
        if data_lf_contact[i] != data_lf_contact[i-1]:
            lf_contact_index_change.append(i)


    # RightFoot Contact Signal #
    data_rf_contact = np.genfromtxt(file_path+'rfoot_contact.txt', delimiter=None, dtype=(float))
    data_rf_contact = data_rf_contact[st_idx:end_idx]
    rf_contact_index_change = []
    rf_contact_index_change.append(0)    
    rf_contact_index_change.append(1)        
    for i in range(2, len(data_x)):
        if data_rf_contact[i] != data_rf_contact[i-1]:
            rf_contact_index_change.append(i)          

    axes = plt.gca()
    
    ### FULL / TRIM ###
    '''comment out the next four lines and set the two values for a limited graph'''
    fullGraph = True # -> full graph
    # fullGraph = False # -> limit time
    setXMin = 5 # set x minimum value
    setXMax = 9 # set x maximum value
    xMinIndex = 1
    xMaxIndex = 100
    if fullGraph:
        xMinIndex = st_idx
        xMaxIndex = end_idx
    # SET THE X MIN & X MAX #
    else:
        for i in range(len(data_x)):
            if data_x[i]>=setXMin:
                xMinIndex = i
                break
        for j in range(len(data_x)):
            if data_x[j]<=setXMax:
                xMaxIndex = j
        data_x = data_x[xMinIndex:xMaxIndex]
    
    ## plot command/jpos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))

    fig.canvas.set_window_title('jtorque (left_leg)')
    for i in range(1,num_leg_joint + 1,1):
        ax1 = plt.subplot(num_leg_joint, 1, i)
        # plt.plot(data_x, data_filter_cmd[st_idx:end_idx,i-1], "c-", linewidth=2.7)
        plt.plot( \
                # data_x, scale[i-1] * data_motor_current[st_idx:end_idx, i-1], "k-", \
                data_x, data_torque[st_idx:end_idx,i-1], "b-",\
                data_x, data_cmd[st_idx:end_idx, i-1], "r-")

        # plt.legend(('command', 'pos'), loc='upper left')
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

        # Plot Left Foot Contact 
        for j in range(1, len(lf_contact_index_change)):
            t_start = data_x[lf_contact_index_change[j-1]]
            t_end = data_x[lf_contact_index_change[j]]            
            y_start = data_lf_contact[lf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0 * 0.9
            y_end = data_lf_contact[lf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0 * 0.9            

            # Plot Square Wave
            plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')

        # Plot Right Foot Contact 
        for j in range(1, len(rf_contact_index_change)):
            t_start = data_x[rf_contact_index_change[j-1]]
            t_end = data_x[rf_contact_index_change[j]]            
            y_start = data_rf_contact[rf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0
            y_end = data_rf_contact[rf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0            

            # Plot Square Wave
            plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')



        plt.grid(True)
    plt.xlabel('time (sec)')
    
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1


    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('jtorque (right_leg)')
    for i in range(1,num_leg_joint+1,1):
        ax1 = plt.subplot(num_leg_joint, 1, i)
        # plt.plot(data_x, data_filter_cmd[st_idx:end_idx,i-1+3], "c-", linewidth=2.7)
        plt.plot(\
                # data_x, scale[i-1] * data_motor_current[st_idx:end_idx, i-1 + 3], "k-", \
                data_x, data_cmd[st_idx:end_idx,i-1 + num_leg_joint], "r-" , \
                data_x, data_torque[st_idx:end_idx,i-1 + num_leg_joint], "b-")

        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

        # Plot Left Foot Contact 
        for j in range(1, len(lf_contact_index_change)):
            t_start = data_x[lf_contact_index_change[j-1]]
            t_end = data_x[lf_contact_index_change[j]]            
            y_start = data_lf_contact[lf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0 * 0.9
            y_end = data_lf_contact[lf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0 * 0.9            

            # Plot Square Wave
            plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')

        # Plot Right Foot Contact 
        for j in range(1, len(rf_contact_index_change)):
            t_start = data_x[rf_contact_index_change[j-1]]
            t_end = data_x[rf_contact_index_change[j]]            
            y_start = data_rf_contact[rf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0
            y_end = data_rf_contact[rf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0            

            # Plot Square Wave
            plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')


        plt.grid(True)
    plt.xlabel('time (sec)')
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

   
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('qddot (floating)')
    for i in range(1,7,1):
        ax1 = plt.subplot(6, 1, i)
        # plt.plot(data_x, data_qddot_cmd[xMinIndex:xMaxIndex, i-1], "r-")
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
    plt.xlabel('time (sec)')
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('qddot (left)')
    for i in range(1,num_leg_joint + 1,1):
        ax1 = plt.subplot(num_leg_joint, 1, i)
        # plt.plot(data_x, data_qddot_cmd[xMinIndex:xMaxIndex, i-1 + 6], "r-")
        plt.grid(True)
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
    plt.xlabel('time (sec)')
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('qddot (right)')
    for i in range(1, num_leg_joint + 1,1):
        ax1 = plt.subplot(num_leg_joint, 1, i)
        # plt.plot(data_x, data_qddot_cmd[xMinIndex:xMaxIndex, i-1 + 9], "r-")
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
    plt.xlabel('time (sec)')
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    

if __name__=="__main__":
    create_figures()
    plt.show()

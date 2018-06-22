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

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index


    file_path = os.getcwd() + "/../../experiment_data_check/"


    # data_minj_pos_des = \
    # np.genfromtxt(file_path+'minj_pos.txt', delimiter=None, dtype=(float))
    # data_minj_vel_des = \
    # np.genfromtxt(file_path+'minj_vel.txt', delimiter=None, dtype=(float))
    # data_minj_acc_des = \
    # np.genfromtxt(file_path+'minj_acc.txt', delimiter=None, dtype=(float))

    ## read files
    data_rfoot_pos_des = \
    np.genfromtxt(file_path+'rfoot_pos_des.txt', delimiter=None, dtype=(float))
    data_rfoot_pos = \
    np.genfromtxt(file_path+'rfoot_pos.txt', delimiter=None, dtype=(float))
    data_lfoot_pos_des = \
    np.genfromtxt(file_path+'lfoot_pos_des.txt', delimiter=None, dtype=(float))
    data_lfoot_pos = \
    np.genfromtxt(file_path+'lfoot_pos.txt', delimiter=None, dtype=(float))
    data_jjpos_rfoot_pos = \
    np.genfromtxt(file_path+'jjpos_rfoot_pos.txt', delimiter=None, dtype=(float))
    data_jjpos_lfoot_pos = \
    np.genfromtxt(file_path+'jjpos_lfoot_pos.txt', delimiter=None, dtype=(float))
 
    data_LED = \
            np.genfromtxt(file_path+'LED_Pos.txt', delimiter=None, dtype=(float))
    data_LED_kin = \
            np.genfromtxt(file_path+'LED_Kin_Pos.txt', delimiter=None, dtype=(float))
    #data_foot_vel_des = \
    #np.genfromtxt(file_path+'rfoot_vel_des.txt', delimiter=None, dtype=(float))
    data_rfoot_vel = \
    np.genfromtxt(file_path+'rfoot_vel.txt', delimiter=None, dtype=(float))
    data_rfoot_acc_des = \
    np.genfromtxt(file_path+'rfoot_acc_des.txt', delimiter=None, dtype=(float))
    data_lfoot_acc_des = \
    np.genfromtxt(file_path+'lfoot_acc_des.txt', delimiter=None, dtype=(float))


    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))
    st_idx = 1
    end_idx = len(data_x) - 10
    data_x = data_x[st_idx:end_idx]

    rfoot_LED_idx = [6, 7];
    lfoot_LED_idx = [11, 12];
    data_LED_rfoot = ((data_LED[:, 3*rfoot_LED_idx[0]:3*rfoot_LED_idx[0]+3]) \
            + (data_LED[:, 3*rfoot_LED_idx[1]:3*rfoot_LED_idx[1]+3]))/2.;

    data_LED_lfoot = ((data_LED[:, 3*lfoot_LED_idx[0]:3*lfoot_LED_idx[0]+3]) \
            + (data_LED[:, 3*lfoot_LED_idx[1]:3*lfoot_LED_idx[1]+3]))/2.;

    data_LED_rfoot_out = (data_LED[:, 3*rfoot_LED_idx[0]:3*rfoot_LED_idx[0]+3]) 
    data_LED_lfoot_out = (data_LED[:, 3*lfoot_LED_idx[0]:3*lfoot_LED_idx[0]+3])
    data_LED_rfoot_in = (data_LED[:, 3*rfoot_LED_idx[1]:3*rfoot_LED_idx[1]+3]) 
    data_LED_lfoot_in = (data_LED[:, 3*lfoot_LED_idx[1]:3*lfoot_LED_idx[1]+3])




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

    ## plot rfoot pos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('right foot pos')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_rfoot_pos_des[st_idx:end_idx,i-1], "r-")
        # plt.plot(data_x, data_minj_pos_des[st_idx:end_idx, i-1], color="black", linewidth=1.5)
        plt.plot(data_x, data_jjpos_rfoot_pos[st_idx:end_idx, i-1], color="black", linewidth=1.5)
        plt.plot(data_x, data_rfoot_pos[st_idx:end_idx,i-1], "b-")

        plt.plot(data_x, data_LED_rfoot[st_idx:end_idx, i-1] - data_LED_lfoot[st_idx:end_idx, i-1], \
                linewidth=2.0, color='orange')
        plt.plot(data_x, data_LED_rfoot_out[st_idx:end_idx, i-1] - data_LED_lfoot[st_idx:end_idx, i-1], \
                linewidth=1.0, color="indigo")
        plt.plot(data_x, data_LED_rfoot_in[st_idx:end_idx, i-1] - data_LED_lfoot[st_idx:end_idx, i-1], \
                linewidth=1.0, color="indigo")
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
            plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


        # Plot Right Foot Contact 
        for j in range(1, len(rf_contact_index_change)):
            t_start = data_x[rf_contact_index_change[j-1]]
            t_end = data_x[rf_contact_index_change[j]]            
            y_start = data_rf_contact[rf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0
            y_end = data_rf_contact[rf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0            

            # Plot Square Wave
            plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
            plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')




        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1    

    ## plot lfoot pos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('left foot pos')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_lfoot_pos_des[st_idx:end_idx,i-1], "r-")       
        plt.plot(data_x, data_jjpos_lfoot_pos[st_idx:end_idx, i-1], color="black", linewidth=1.5)
        plt.plot(data_x, data_lfoot_pos[st_idx:end_idx,i-1], "b-")

        plt.plot(data_x, data_LED_lfoot[st_idx:end_idx, i-1] \
                - data_LED_rfoot[st_idx:end_idx, i-1], \
                linewidth=2.0, color='orange')

        plt.plot(data_x, data_LED_lfoot_out[st_idx:end_idx, i-1] \
                - data_LED_rfoot[st_idx:end_idx, i-1], \
                linewidth=1.0, color="indigo")

        plt.plot(data_x, data_LED_lfoot_in[st_idx:end_idx, i-1] \
                - data_LED_rfoot[st_idx:end_idx, i-1], \
                linewidth=1.0, color="indigo")
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
            plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


        # Plot Right Foot Contact 
        for j in range(1, len(rf_contact_index_change)):
            t_start = data_x[rf_contact_index_change[j-1]]
            t_end = data_x[rf_contact_index_change[j]]            
            y_start = data_rf_contact[rf_contact_index_change[j-1]] * ax1.get_ylim()[1]/2.0
            y_end = data_rf_contact[rf_contact_index_change[j]] * ax1.get_ylim()[1]/2.0            

            # Plot Square Wave
            plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
            plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')




        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1    


    ## plot foot vel
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('foot vel')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        #plt.plot(data_x, data_foot_vel_des[st_idx:end_idx,i-1], "r-", \
        #         data_x, data_rfoot_vel[st_idx:end_idx,i-1], "b-")
        plt.plot(data_x, data_rfoot_vel[st_idx:end_idx,i-1], "b-")
        # plt.plot(data_x, data_minj_vel_des[st_idx:end_idx, i-1], color="black", linewidth=1.5)
        
        # plt.legend(('command', 'pos'), loc='upper left')
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1    

    ## plot foot acc
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('rfoot acc')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_rfoot_acc_des[st_idx:end_idx,i-1], "r-")
        # plt.plot(data_x, data_minj_acc_des[st_idx:end_idx, i-1], color="black", linewidth=1.5)

        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1    

    ## plot foot acc
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('lfoot acc')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_lfoot_acc_des[st_idx:end_idx,i-1], "r-")
        # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1    





if __name__ == "__main__":
    create_figures()
    plt.show()

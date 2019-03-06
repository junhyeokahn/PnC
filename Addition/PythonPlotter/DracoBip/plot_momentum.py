import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index


    file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

    ## read files
    data_momentum_pred= \
            np.genfromtxt(file_path+'debug_pred.txt', delimiter=None, dtype=(float))
    data_momentum_obs= \
            np.genfromtxt(file_path+'debug_obs.txt', delimiter=None, dtype=(float))
    data_momentum_upd= \
            np.genfromtxt(file_path+'debug_upd.txt', delimiter=None, dtype=(float))
    data_momentum_true= \
            np.genfromtxt(file_path+'debug_true.txt', delimiter=None, dtype=(float))
    data_rfoot_ati= \
            np.genfromtxt(file_path+'debug_rfoot_ati.txt', delimiter=None, dtype=(float))
    data_lfoot_ati= \
            np.genfromtxt(file_path+'debug_lfoot_ati.txt', delimiter=None, dtype=(float))
    data_rfoot_ati_raw= \
            np.genfromtxt(file_path+'debug_rfoot_ati_raw.txt', delimiter=None, dtype=(float))
    data_lfoot_ati_raw= \
            np.genfromtxt(file_path+'debug_lfoot_ati_raw.txt', delimiter=None, dtype=(float))

    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))
    st_idx = 5
    end_idx = len(data_x)
    # end_idx = st_idx + 2000
    data_x = data_x[st_idx:end_idx]


    # PHASE MARKER #
    data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    data_phse = data_phse[st_idx:end_idx]
    # get phase.txt data #
    phseChange = []
    for i in range(0,len(data_x)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i+1)
        else:
            pass

    axes = plt.gca()

    ## plot c
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('c')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_momentum_pred[st_idx:end_idx,i-1], "g--", linewidth=3, label='pred')
        plt.plot(data_x, data_momentum_obs[st_idx:end_idx,i-1], "b--", linewidth=3, label='obs')
        plt.plot(data_x, data_momentum_upd[st_idx:end_idx,i-1], "r-", linewidth=1.5, label='update')
        plt.plot(data_x, data_momentum_true[st_idx:end_idx,i-1], "k-", linewidth=1.5, label='true')

       # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
        plt.legend()
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    ## plot l
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('l')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_momentum_pred[st_idx:end_idx,i+2], "g--", linewidth=3, label='pred')
        plt.plot(data_x, data_momentum_obs[st_idx:end_idx,i+2], "b--", linewidth=3, label='obs')
        plt.plot(data_x, data_momentum_upd[st_idx:end_idx,i+2], "r-", linewidth=1.5, label='update')
        plt.plot(data_x, data_momentum_true[st_idx:end_idx,i+2], "k-", linewidth=1.5, label='true')

       # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
        plt.legend()
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    ## plot k
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('k')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_momentum_pred[st_idx:end_idx,i+5], "g--", linewidth=3, label='pred')
        plt.plot(data_x, data_momentum_obs[st_idx:end_idx,i+5], "b--", linewidth=3, label='obs')
        plt.plot(data_x, data_momentum_upd[st_idx:end_idx,i+5], "r-", linewidth=1.5, label='update')
        plt.plot(data_x, data_momentum_true[st_idx:end_idx,i+5], "k-", linewidth=1.5, label='true')

       # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
        plt.legend()
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    ## plot ati
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('ati')
    for i in range(1,7,1):
        ax1 = plt.subplot(6, 1, i)
        plt.plot(data_x, data_rfoot_ati[st_idx:end_idx, i-1], "b-", linewidth=1.5, label="rfoot ati")
        plt.plot(data_x, data_lfoot_ati[st_idx:end_idx, i-1], "r-", linewidth=1.5, label="lfoot ati")
        # plt.plot(data_x, data_rfoot_ati_raw[st_idx:end_idx, i-1], "c--", linewidth=3, label="rfoot ati raw")
        # plt.plot(data_x, data_lfoot_ati_raw[st_idx:end_idx, i-1], "m--", linewidth=3, label="lfoot ati raw")

       # phase marker #
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.grid(True)
        plt.legend()
    plt.xlabel('time (sec)')

if __name__ == "__main__":
    create_figures()
    plt.show()

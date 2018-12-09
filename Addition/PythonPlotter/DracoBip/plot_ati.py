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


    file_path = os.getcwd() + "/../../../ExperimentDataCheck/"

    ## read files
    data_rfoot_ati= \
            np.genfromtxt(file_path+'rfoot_ati.txt', delimiter=None, dtype=(float))
    data_rfoot_ati_norm = np.zeros(len(data_rfoot_ati))

    # __import__('ipdb').set_trace()
    # exit()
    for i in range(len(data_rfoot_ati)):
        data_rfoot_ati_norm[i] = np.linalg.norm(data_rfoot_ati[i, 3:6])

    data_lfoot_ati = \
            np.genfromtxt(file_path+'lfoot_ati.txt', delimiter=None, dtype=(float))

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
            phseChange.append(i)
        else:
            pass


    axes = plt.gca()

    ## plot rfoot pos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('right foot ati')
    for i in range(1,8,1):
        if i != 7:
            ax1 = plt.subplot(7, 1, i)
            plt.plot(data_x, data_rfoot_ati[st_idx:end_idx,i-1], \
                    "r-", linewidth = 3)
        else:
            plt.subplot(7, 1, 7)
            plt.plot(data_x, data_rfoot_ati_norm[st_idx:end_idx], "b-", linewidth = 3)

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

    ## plot lfoot pos
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('left foot ati')
    for i in range(1,7,1):
        ax1 = plt.subplot(7, 1, i)
        plt.plot(data_x, data_lfoot_ati[st_idx:end_idx,i-1],\
                "r-", linewidth=3)

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

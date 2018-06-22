import sys
import importlib

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1
PLOT_CONFIG = PLOT_VERTICALLY

# List of valid figures to plot. Must be the names of the plotting script without the .py extension
valid_figures = ["PlotStateTrajectory", "PlotInputTrajectory"]
figures_to_plot = {}
screen_resolution_width = 2920
screen_resolution_height = 1280
num_plot_scripts = 0

def count_num_plot_scripts():
    global num_plot_scripts, figures_to_plot
    for item in sys.argv:
        if item in valid_figures:
            figures_to_plot[item] = True
            module = importlib.import_module(item)
            num_plot_scripts += 1


def create_all_figures():
    global valid_figures, figures_to_plot, screen_resolution_width, screen_resolution_height, num_plot_scripts
    # Initialize row index and figure number
    col_index = 0
    row_index = 0
    figure_number = 1

    for key in figures_to_plot:
        print ("Plotting", key)
        # Import plotting scripts on the fly
        module = importlib.import_module(key)
        # Prepare the sizes of the subfigure

        use_subfigure_width = screen_resolution_width / module.num_figures
        use_subfigure_height = screen_resolution_height / num_plot_scripts

        if PLOT_CONFIG == PLOT_VERTICALLY:
            use_subfigure_width = screen_resolution_width / num_plot_scripts
            use_subfigure_height = screen_resolution_height / module.num_figures

        # Create the subfigure
        module.create_figures(subfigure_width=use_subfigure_width, subfigure_height=use_subfigure_height, starting_figure_no=figure_number, starting_col_index=col_index, starting_row_index=row_index, plot_configuration=PLOT_CONFIG)
        ## Increment the figure number and index
        figure_number += (module.num_figures)

        if PLOT_CONFIG == PLOT_VERTICALLY:
            col_index += 1
        elif PLOT_CONFIG == PLOT_HORIZONTALLY:
            row_index += 1


if __name__ == "__main__":
    count_num_plot_scripts()
    create_all_figures()
    plt.show()


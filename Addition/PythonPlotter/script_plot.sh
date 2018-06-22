#!/bin/bash
plot_these=(PlotStateTrajectory
            PlotInputTrajectory)

python plot_multiple.py  ${plot_these[*]}

clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentData';

%%
fig = fn_open_figures(3);

Time = fn_read_file(data_path, 'test_time', 1);
CoMPosAct = fn_read_file(data_path, 'test_com_pos', 3);
CoMVelAct = fn_read_file(data_path, 'test_com_vel', 3);
CoMPosDes = fn_read_file(data_path, 'test_com_pos_des', 3);
CoMVelDes = fn_read_file(data_path, 'test_com_vel_des', 3);

%% Plot
for i = 1:3
    figure(fig(i))
    subplot(2,1,1)
    hold on
    plot(Time, CoMPosDes(i, :), 'r', 'linewidth', 3);
    plot(Time, CoMPosAct(i, :), 'b', 'linewidth', 3);
    grid on
    hold off
    subplot(2,1,2)
    hold on
    plot(Time, CoMVelDes(i, :), 'r', 'linewidth', 3);
    plot(Time, CoMVelAct(i, :), 'b', 'linewidth', 3);
    hold off
    grid on
end
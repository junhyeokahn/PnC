clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentDataCheck';
fn_path = '/home/apptronik/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/home/apptronik/Repository/PnC/ExperimentDataCheck';
%%
fig = fn_open_figures(3);

Time = fn_read_file(data_path, 'Time', 1);
RfPosAct = fn_read_file(data_path, 'rf_pos_act_debug', 3);
RfPosDes = fn_read_file(data_path, 'rf_pos_des_debug', 3);
RfVelAct = fn_read_file(data_path, 'rf_vel_act_debug', 3);
RfVelDes = fn_read_file(data_path, 'rf_vel_des_debug', 3);
RfAccDes = fn_read_file(data_path, 'rf_acc_des_debug', 3);

%% Plot
start_idx = 1;
end_idx = length(Time)-2;
for i = 1:3
    figure(fig(i))
    subplot(3,1,1)
    hold on
    plot(Time(start_idx:end_idx), RfPosDes(i, start_idx:end_idx), 'r', 'linewidth', 3);
    plot(Time(start_idx:end_idx), RfPosAct(i, start_idx:end_idx), 'b', 'linewidth', 1);
    ylabel('rf pos');
    grid on
    hold off
    subplot(3,1,2)
    hold on
    plot(Time(start_idx:end_idx), RfVelDes(i, start_idx:end_idx), 'r', 'linewidth', 3);
    plot(Time(start_idx:end_idx), RfVelAct(i, start_idx:end_idx), 'b', 'linewidth', 1);
    hold off
    grid on
    subplot(3,1,3)
    plot(Time(start_idx:end_idx), RfAccDes(i, start_idx:end_idx), 'r', 'linewidth', 3);
    grid on
end

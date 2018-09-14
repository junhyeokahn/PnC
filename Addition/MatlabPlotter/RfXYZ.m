clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentDataCheck';

%%
fig = fn_open_figures(3);

Time = fn_read_file(data_path, 'Time', 1);
RfPosAct = fn_read_file(data_path, 'rf_pos_act_debug', 3);
RfPosDes = fn_read_file(data_path, 'rf_pos_des_debug', 3);
RfVelAct = fn_read_file(data_path, 'rf_vel_act_debug', 3);
RfVelDes = fn_read_file(data_path, 'rf_vel_des_debug', 3);
RfAccDes = fn_read_file(data_path, 'rf_acc_des_debug', 3);

%% Plot
for i = 1:3
    figure(fig(i))
    subplot(3,1,1)
    hold on
    plot(Time, RfPosDes(i, :), 'r', 'linewidth', 3);
    plot(Time, RfPosAct(i, :), 'b', 'linewidth', 3);
    ylabel('rf pos');
    grid on
    hold off
    subplot(3,1,2)
    hold on
    plot(Time, RfVelDes(i, :), 'r', 'linewidth', 3);
    plot(Time, RfVelDes(i, :), 'b', 'linewidth', 3);
    hold off
    grid on
    subplot(3,1,3)
    plot(Time, RfAccDes(i, :), 'r', 'linewidth', 3);
    grid on
end

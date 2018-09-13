clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentDataCheck';

%%
targetJointIdx = 1:10;

numJoint = 10;
numTarget = length(targetJointIdx);
fig = fn_open_figures(numTarget);

Time = fn_read_file(data_path, 'Time', 1);
Q = fn_read_file(data_path, 'q_des_debug', numJoint);
Qdot = fn_read_file(data_path, 'qdot_des_debug', numJoint);
Q_act = fn_read_file(data_path, 'q_act_debug', numJoint);
Qdot_act = fn_read_file(data_path, 'qdot_act_debug', numJoint);
Qddot = fn_read_file(data_path, 'qddot_des_debug', numJoint);

startIdx = 5;
endIdx = length(Q);
%% Plot
for i = 1:numTarget
    figure(fig(i))
    subplot(3, 1, 1)
    hold on
    plot(Time, Q(i, :),'r', 'linewidth', 3);
    plot(Time, Q_act(i, :),'b', 'linewidth', 3);
    hold off
    grid on
    subplot(3, 1, 2)
    hold on
    plot(Time, Qdot(i, :),'r', 'linewidth', 3);
    plot(Time, Qdot_act(i, :),'b', 'linewidth', 3);
    hold off
    grid on
    subplot(3, 1, 3)
    plot(Time, Qddot(i, :), 'linewidth', 3);
    hold off
    grid on
end
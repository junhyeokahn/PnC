clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentData';

%%
targetJointIdx = 1:16;

numJoint = 16;
numTarget = length(targetJointIdx);
fig = fn_open_figures(numTarget);

Time = fn_read_file(data_path, 'test_time', 1);
Q = fn_read_file(data_path, 'test_q', numJoint);
Qdot = fn_read_file(data_path, 'test_qdot', numJoint);

startIdx = 5;
endIdx = length(Time);
%% Plot
for i = 1:numTarget
    figure(fig(i))
    subplot(2, 1, 1)
    plot(Time, Q(i, :), 'linewidth', 3);
    hold off
    grid on
    subplot(2, 1, 2)
    plot(Time, Qdot(i, :), 'linewidth', 3);
    hold off
    grid on
end
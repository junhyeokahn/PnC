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

% Time = fn_read_file(data_path, 'test_time', 1);
% Q = fn_read_file(data_path, 'test_q', numJoint);
% Qdot = fn_read_file(data_path, 'test_qdot', numJoint);
Q = fn_read_file(data_path, 'q_worldnode', numJoint);
Qdot = fn_read_file(data_path, 'qdot_worldnode', numJoint);
Frc = fn_read_file(data_path, 'frc_worldnode', numJoint);

startIdx = 5;
endIdx = length(Q);
%% Plot
for i = 1:numTarget
    figure(fig(i))
    subplot(3, 1, 1)
    plot(Q(i, :), 'linewidth', 3);
    hold off
    grid on
    subplot(3, 1, 2)
    plot(Qdot(i, :), 'linewidth', 3);
    hold off
    grid on
    subplot(3, 1, 3)
    plot(Frc(i, :), 'linewidth', 3);
    hold off
    grid on
end
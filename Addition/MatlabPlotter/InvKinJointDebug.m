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

Q = fn_read_file(data_path, 'ikp_qsol', numJoint);
Qdot = fn_read_file(data_path, 'ikp_q_dot_sol', numJoint);

startIdx = 5;
endIdx = length(Q);
%% Plot
for i = 1:numTarget
    figure(fig(i))
    subplot(2, 1, 1)
    plot(Q(i, :), 'o-', 'linewidth', 3);
    hold off
    grid on
    subplot(2, 1, 2)
    plot(Qdot(i, :), 'o-', 'linewidth', 3);
    hold off
    grid on
end
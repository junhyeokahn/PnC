clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentData';

%%
targetJointIdx = 1:36;

numJoint = 36;
numTarget = length(targetJointIdx);
fig = fn_open_figures(numTarget);

% Time = fn_read_file(data_path, 'Time', 1);
TaskPosDes = fn_read_file(data_path, 'q_des', numJoint);
TaskVelDes = fn_read_file(data_path, 'qdot_des', numJoint);
TaskPosAct = fn_read_file(data_path, 'q_act', numJoint);
TaskVelAct = fn_read_file(data_path, 'qdot_act', numJoint);
Trq = fn_read_file(data_path, 'trq', numJoint);
Time = [1:length(TaskPosDes)];
startIdx = 5;
endIdx = length(Time)-10;
%% Plot
for i = 1:numTarget
    figure(fig(i))
    subplot(3,1,1)
    hold on
    plot(Time(startIdx:endIdx), TaskPosDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), TaskPosAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 2);
    grid on
    hold off
    subplot(3,1,2)
    hold on
    plot(Time(startIdx:endIdx), TaskVelDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), TaskVelAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 2);
    grid on
    hold off
    subplot(3,1,3)
    hold on
    plot(Time(startIdx:endIdx), Trq(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    grid on
end
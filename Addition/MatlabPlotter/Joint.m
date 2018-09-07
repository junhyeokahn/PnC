clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentDataCheck';

%%
% targetJointIdx = [17,25]; %Left Shoulder, Right Shoulder
% targetJointIdx = [4, 10];
targetJointIdx = 1:10;

numJoint = 10;
numTarget = length(targetJointIdx);
fig = fn_open_figures(numTarget);

Time = fn_read_file(data_path, 'Time', 1);
TaskPosDes = fn_read_file(data_path, 'JointTaskPosDes', numJoint);
TaskVelDes = fn_read_file(data_path, 'JointTaskVelDes', numJoint);
TaskPosAct = fn_read_file(data_path, 'JointTaskPosAct', numJoint);
TaskVelAct = fn_read_file(data_path, 'JointTaskVelAct', numJoint);
debug_cnst_frc = fn_read_file(data_path, 'ConstFrc', 16);
Cmd = fn_read_file(data_path, 'TorqueCommand', numJoint);

startIdx = 5;
endIdx = length(Time);
%% Plot
for i = 1:numTarget
    figure(fig(i))
    subplot(3,1,1)
    hold on
    plot(Time(startIdx:endIdx), TaskPosDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), TaskPosAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 3);
    grid on
    hold off
    subplot(3,1,2)
    hold on
    plot(Time(startIdx:endIdx), TaskVelDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), TaskVelAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 3);
    grid on
    hold off
    subplot(3,1,3)
    hold on
    plot(Time(startIdx:endIdx), debug_cnst_frc(targetJointIdx(i)+6, startIdx:endIdx),'r', 'linewidth', 3);
    hold off
    grid on
end
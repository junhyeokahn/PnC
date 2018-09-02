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
targetJointIdx = 1:6;
targetJointIdx = 7:16;

numJoint = 16;
numTarget = length(targetJointIdx);
fig = fn_open_figures(numTarget);

Time = fn_read_file(data_path, 'Time', 1);
TaskPosDes = fn_read_file(data_path, 'QSol', numJoint);
TaskVelDes = fn_read_file(data_path, 'QdotSol', numJoint);
TaskPosAct = fn_read_file(data_path, 'QAct', numJoint);
TaskVelAct = fn_read_file(data_path, 'QdotAct', numJoint);
debugQdot = fn_read_file(data_path, 'debugQdot', numJoint);

startIdx = 5;
endIdx = length(Time)-200;
%% Plot
for i = 1:numTarget
    figure(fig(i))
    subplot(2,1,1)
    hold on
    plot(Time(startIdx:endIdx), TaskPosDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), TaskPosAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 3);
    grid on
    hold off
    subplot(2,1,2)
    hold on
    plot(Time(startIdx:endIdx), TaskVelDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), TaskVelAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 3);
    grid on
    hold off
%     hold on
%     plot(Time(startIdx:endIdx), debugQdot(targetJointIdx(i), startIdx:endIdx), 'g', 'linewidth', 3);
%     hold off
end
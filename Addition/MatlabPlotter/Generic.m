clear all
clc 
close all

%% 
fn_path = '/home/apptronik/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/home/apptronik/Repository/PnC/ExperimentDataCheck';

%%
targetJointIdx = 6:10;
numJoint = 10;
numTarget = length(targetJointIdx);
fig = fn_open_figures(numTarget);

Time = fn_read_file(data_path, 'Time', 1);
JPosDes = fn_read_file(data_path, 'JPosDes', numJoint);
JVelDes = fn_read_file(data_path, 'JVelDes', numJoint);
JTrqDes = fn_read_file(data_path, 'JTrqDes', numJoint);
JPosAct = fn_read_file(data_path, 'JPosAct', numJoint);
JVelAct = fn_read_file(data_path, 'JVelAct', numJoint);
JTrqAct = fn_read_file(data_path, 'JTrqAct', numJoint);

startIdx = 5;
endIdx = length(Time)-5;
%% Plot
for i = 1:numTarget
    figure(fig(i))
    subplot(3,1,1)
    hold on
    plot(Time(startIdx:endIdx), JPosDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), JPosAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 3);
    grid on
    hold off
    subplot(3,1,2)
    hold on
    plot(Time(startIdx:endIdx), JVelDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), JVelAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 3);
    grid on
    hold off
    subplot(3,1,3)
    hold on
    plot(Time(startIdx:endIdx), JTrqDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), JTrqAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 3);
    grid on
    hold off
end
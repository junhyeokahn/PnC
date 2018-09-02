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
FF = fn_read_file(data_path, 'ff', numJoint);
FB = fn_read_file(data_path, 'fb', numJoint);
errSum = fn_read_file(data_path, 'errSum', numJoint);

startIdx = 5;
endIdx = length(Time)-200;
%% Plot
for i = 1:numTarget
    figure(fig(i))
    subplot(2,1,1)
    hold on
    plot(Time(startIdx:endIdx), FF(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), FB(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 3);
    grid on
    hold off
    subplot(2,1,2)
    plot(Time(startIdx:endIdx), errSum(targetJointIdx(i), startIdx:endIdx),'g', 'linewidth', 3);
    grid on
    hold off
%     hold on
%     plot(Time(startIdx:endIdx), debugQdot(targetJointIdx(i), startIdx:endIdx), 'g', 'linewidth', 3);
%     hold off
end
clear all
clc 
close all

%% 
fn_path = '/Users/junhyeok/Repository/RobotLocomotion/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeok/Repository/RobotLocomotion/ExperimentDataCheck';

%%
fig = fn_open_figures(2);

Time = fn_read_file(data_path, 'Time', 1);
TaskPosDes = fn_read_file(data_path, 'CentroidTaskPosDes', 6);
TaskVelDes = fn_read_file(data_path, 'CentroidTaskVelDes', 6);
TaskPosAct = fn_read_file(data_path, 'CentroidTaskPosAct', 6);
TaskVelAct = fn_read_file(data_path, 'CentroidTaskVelAct', 6);

startIdx = 1;
endIdx = length(Time);
%% Plot
figure(fig(1))
subplot(3,1,1)
hold on
plot(Time(startIdx:endIdx), TaskPosDes(4, startIdx:endIdx), 'r', 'linewidth', 3);
plot(Time(startIdx:endIdx), TaskPosAct(4, startIdx:endIdx), 'b', 'linewidth', 3);
hold off

subplot(3,1,2)
hold on
plot(Time(startIdx:endIdx), TaskPosDes(5, startIdx:endIdx), 'r', 'linewidth', 3);
plot(Time(startIdx:endIdx), TaskPosAct(5, startIdx:endIdx), 'b', 'linewidth', 3);
hold off

subplot(3,1,3)
hold on
plot(Time(startIdx:endIdx), TaskPosDes(6, startIdx:endIdx), 'r', 'linewidth', 3);
plot(Time(startIdx:endIdx), TaskPosAct(6, startIdx:endIdx), 'b', 'linewidth', 3);
hold off

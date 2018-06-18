clear all
clc 
close all

%%
fig = fn_open_figures(2);

%% 
fn_path = '/Users/junhyeok/Repository/RobotLocomotion/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeok/Repository/RobotLocomotion/ExperimentDataCheck';
data_path2 = '/Users/junhyeok/Repository/dynacore/experiment_data_check';

t = fn_read_file(data_path, 'time', 1);
vel = fn_read_file(data_path, 'vel', 6);
pos = fn_read_file(data_path, 'headpos', 3);
ori = fn_read_file(data_path, 'headori', 4);

t2 = fn_read_file(data_path2, 'timed', 1);
vel2 = fn_read_file(data_path2, 'vel', 6);
pos2 = fn_read_file(data_path2, 'headpos', 3);
ori2 = fn_read_file(data_path2, 'headori', 4);

%% Plot
figure(fig(1))
hold on
idx = 4;
plot(t, vel(idx,:), 'linewidth', 3);
plot(t2, vel2(idx,:), 'linewidth', 3);
hold off

figure(fig(2))
idx2 = 3;
idx3 = 2;
subplot(2,1,1)
hold on
plot(t, pos(idx2,:), 'linewidth', 3);
plot(t2, pos2(idx2,:), 'linewidth', 3);
hold off
subplot(2,1,2)
hold on
plot(t, ori(idx3,:), 'linewidth', 3);
plot(t2, ori2(idx3,:), 'linewidth', 3);
hold off
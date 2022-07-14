close all
clear all
clc

addpath("/tmp")
% load latest .mat file created
d = dir("/tmp/*.mat");
[tmp i] = max([d.datenum]);
fprintf('loading %s', d(i).name)
load(d(i).name)


figure(1)
subplot(4,1,1)
plot(c0(1,:))
hold on
grid on
plot(c_ref0(1,:))

subplot(4,1,2)
plot(cdot0(1,:))
grid on

subplot(4,1,3)
plot(c0(3,:))
hold on 
grid on
plot(c_ref0(3,:))

subplot(4,1,4)
plot(cdot0(3,:))
grid on

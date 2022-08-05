close all
clear all
clc

addpath("/tmp")
% load latest .mat file created
d = dir("/tmp/*.mat");
[tmp i] = max([d.datenum]);
fprintf('loading %s', d(i).name)
load(d(i).name)

%% Plot contact positions and velocities
figure
subplot(4,1,1)
plot(c0(1,:))
hold on
grid on
plot(c_ref0(1,:))
legend('act', 'ref')
ylabel('x')

subplot(4,1,2)
plot(cdot0(1,:))
grid on

subplot(4,1,3)
plot(c0(3,:))
hold on 
grid on
plot(c_ref0(3,:))
legend('act', 'ref')
ylabel('z')

subplot(4,1,4)
plot(cdot0(3,:))
grid on

%% Plot com positions and velocities
figure
subplot(2,1,1)
plot(r(2,:))
hold on
plot(r_ref(2,:))
legend('act', 'ref')
grid on
subplot(2,1,2)
grid on
hold on
plot(rdot(2,:))
plot(rdot_ref(2,:))
legend('act', 'ref')




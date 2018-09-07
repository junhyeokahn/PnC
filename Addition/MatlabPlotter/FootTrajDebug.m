clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentData';

%%
fig = fn_open_figures(4);

Time = fn_read_file(data_path, 'tmp_time', 1);
RfPosDes = fn_read_file(data_path, 'tmp_pos_0', 3);
RfVelDes = fn_read_file(data_path, 'tmp_vel_0', 3);
LfPosDes = fn_read_file(data_path, 'tmp_pos_1', 3);
LfVelDes = fn_read_file(data_path, 'tmp_vel_1', 3);

%% Plot
figure(fig(1));
for i = 1:3
    subplot(3,1,i)
    hold on
    plot(Time, RfPosDes(i, :), 'r', 'linewidth', 3);
    grid on
    hold off
    ylabel('rf pos');
    xlabel('time');
end

figure(fig(2));
for i = 1:3
    subplot(3,1,i)
    hold on
    plot(Time, RfVelDes(i, :), 'r', 'linewidth', 3);
    grid on
    hold off
    ylabel('rf vel');
    xlabel('time');
end

figure(fig(3));
for i = 1:3
    subplot(3,1,i)
    hold on
    plot(Time, LfPosDes(i, :), 'r', 'linewidth', 3);
    grid on
    hold off
    ylabel('lf pos');
    xlabel('time');
end

figure(fig(4));
for i = 1:3
    subplot(3,1,i)
    hold on
    plot(Time, LfVelDes(i, :), 'r', 'linewidth', 3);
    grid on
    hold off
    ylabel('lf vel');
    xlabel('time');
end
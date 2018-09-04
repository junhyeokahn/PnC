clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentData';

%%
fig = fn_open_figures(13);

Time = fn_read_file(data_path, 'test_time', 1);
QuatDes = fn_read_file(data_path, 'test_quat_des', 4);
RfPosAct = fn_read_file(data_path, 'test_rf_pos', 3);
RfPosDes = fn_read_file(data_path, 'test_rf_pos_des', 3);
RfQuatAct = fn_read_file(data_path, 'test_rf_quat', 4);
RfVelAct = fn_read_file(data_path, 'test_rf_vel', 6);
LfPosAct = fn_read_file(data_path, 'test_lf_pos', 3);
LfPosDes = fn_read_file(data_path, 'test_lf_pos_des', 3);
LfQuatAct = fn_read_file(data_path, 'test_lf_quat', 4);
LfVelAct = fn_read_file(data_path, 'test_lf_vel', 6);

%% Plot
for i = 1:3
    figure(fig(i))
    subplot(2,1,1)
    hold on
    plot(Time, RfPosDes(i, :), 'r', 'linewidth', 3);
    plot(Time, RfPosAct(i, :), 'b', 'linewidth', 3);
    ylabel('rf pos');
    grid on
    hold off
    subplot(2,1,2)
    hold on
    plot(Time, LfPosDes(i, :), 'r', 'linewidth', 3);
    plot(Time, LfPosAct(i, :), 'b', 'linewidth', 3);
    hold off
    grid on
    ylabel('lf pos');
    xlabel('time');
end

for i = 1:4
    figure(fig(i+3))
    subplot(2,1,1)
    hold on
    plot(Time, QuatDes(i,:), 'r', 'linewidth', 3);
    plot(Time, RfQuatAct(i,:), 'b', 'linewidth', 3);
    grid on
    hold off
    ylabel('rf quat');
    subplot(2,1,2)
    hold on
    plot(Time, QuatDes(i,:), 'r', 'linewidth', 3);
    plot(Time, LfQuatAct(i,:), 'b', 'linewidth', 3);
    ylabel('lf quat');
    hold off
    grid on
    xlabel('time');
end

for i = 1:6
    figure(fig(i+7))
    subplot(2, 1, 1)
    plot(Time, RfVelAct(i,:), 'b', 'linewidth', 3);
    grid on
    ylabel('rf vel')
    subplot(2, 1, 2)
    plot(Time, LfVelAct(i,:), 'b', 'linewidth', 3);
    grid on
    ylabel('lf vel');
    xlabel('time')
end
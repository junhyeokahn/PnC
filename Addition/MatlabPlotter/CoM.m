clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentDataCheck';

%%
fig = fn_open_figures(3);

Time = fn_read_file(data_path, 'Time', 1);
CoMPosDes = fn_read_file(data_path, 'CoMPosDes', 3);
CoMPosAct = fn_read_file(data_path, 'CoMPosAct', 3);
CoMVelDes = fn_read_file(data_path, 'CoMVelDes', 3);
CoMVelAct = fn_read_file(data_path, 'CoMVelAct', 3);
CoMPosSol = fn_read_file(data_path, 'CoMPosSol', 3);

startIdx = 3;
endIdx = length(Time)-200;
%% Plot
for i = 1:3
    figure(fig(i))
    subplot(2,1,1)
    hold on
    plot(Time(startIdx:endIdx), CoMPosDes(i, startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), CoMPosAct(i, startIdx:endIdx),'b', 'linewidth', 3);
    plot(Time(startIdx:endIdx), CoMPosSol(i, startIdx:endIdx),'g', 'linewidth', 3);    
    grid on
    hold off
    subplot(2,1,2)
    hold on
    plot(Time(startIdx:endIdx), CoMVelDes(i, startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), CoMVelAct(i, startIdx:endIdx),'b', 'linewidth', 3);
    hold off
    grid on
end
clear all
clc 
close all

%% 
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentData';

%%
fig = fn_open_figures(5);

Time = fn_read_file(data_path, 't', 1);

r = fn_read_file(data_path, 'r_des', 3);
rdot = fn_read_file(data_path, 'rdot_des', 3);

k = fn_read_file(data_path, 'k_des', 3);
kdot = fn_read_file(data_path, 'kdot_des', 3);

l = fn_read_file(data_path, 'l_des', 3);
ldot = fn_read_file(data_path, 'ldot_des', 3);

lf = fn_read_file(data_path, 'lf_des', 3);
lfdot = fn_read_file(data_path, 'lfdot_des', 3);

rf = fn_read_file(data_path, 'rf_des', 3);
rfdot = fn_read_file(data_path, 'rfdot_des', 3);

startIdx = 1;
endIdx = length(Time)-5;
%% Plot
figure(fig(1))
for i = 1 : 3
    subplot(3, 2, i*2-1)
    plot(Time(startIdx:endIdx), r(i, startIdx:endIdx), 'linewidth', 3);
    if i == 1
        title('com pos')
    end
    grid on
    subplot(3, 2, i*2)
    plot(Time(startIdx:endIdx), rdot(i, startIdx:endIdx), 'linewidth', 3);
    if i == 1
        title('com vel')
    end
    grid on
end

figure(fig(2))
for i = 1 : 3
    subplot(3, 2, i*2-1)
    plot(Time(startIdx:endIdx), k(i, startIdx:endIdx), 'linewidth', 3);
    if i == 1
        title('ang mom')
    end
    grid on
    subplot(3, 2, i*2)
    plot(Time(startIdx:endIdx), kdot(i, startIdx:endIdx), 'linewidth', 3);
    if i == 1
        title('ang mom dot')
    end
    grid on
end

figure(fig(3))
for i = 1 : 3
    subplot(3, 2, i*2-1)
    hold on
    plot(Time(startIdx:endIdx), l(i, startIdx:endIdx), 'linewidth', 3);
    plot(Time(startIdx:endIdx), 36.81*rdot(i, startIdx:endIdx), 'r', 'linewidth', 1);
    hold off
    if i == 1
        title('lin mom')
    end
    grid on
    subplot(3, 2, i*2)
    hold on
    plot(Time(startIdx:endIdx), ldot(i, startIdx:endIdx), 'linewidth', 3);
    if i == 1
        title('lin mom dot')
    end
    grid on
end

figure(fig(4))
for i = 1 : 3
    subplot(3, 2, i*2-1)
    plot(Time(startIdx:endIdx), rf(i, startIdx:endIdx), 'linewidth', 3);
    if i == 1
        title('rf pos')
    end
    grid on
    subplot(3, 2, i*2)
    plot(Time(startIdx:endIdx), rfdot(i, startIdx:endIdx), 'linewidth', 3);
    if i == 1
        title('rf vel')
    end
    grid on
end

figure(fig(5))
for i = 1 : 3
    subplot(3, 2, i*2-1)
    plot(Time(startIdx:endIdx), lf(i, startIdx:endIdx), 'linewidth', 3);
    if i == 1
        title('lf pos')
    end
    grid on
    subplot(3, 2, i*2)
    plot(Time(startIdx:endIdx), lfdot(i, startIdx:endIdx), 'linewidth', 3);
    if i == 1
        title('lf vel')
    end
    grid on
end
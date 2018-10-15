close all; clc; clear;

%% Read File and Construct Matrix
addpath('/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions/yaml')
addpath('/Users/junhyeokahn/Repository/PnC/Addition/RoboticsUtils/MATLAB/')
f = YAML.read('/Users/junhyeokahn/Repository/PnC/Config/FixedDraco/SysID/MASS.yaml');
num_joint = 5;
num_data = 100;
% Selection Matrix
S = [0, 0, 1, 0, 0, 0;
     1, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0];
 
% Decision Variable
dm = sym('m', [5, 1]);
dr = sym('r', [5, 3]);

%% Data Pre-processing
data_name_list = fieldnames(f);
joint_name_list = fieldnames(f.data0);
num_data = length(data_name_list);
for i = 1 : num_data
    
    for j = 1 : 4
        T_j_j{j} = f.(data_name_list{i}).(joint_name_list{j}).T_j_j;
        T_j_com{j} = f.(data_name_list{i}).(joint_name_list{j}).T_j_com;
        R_w_com{j} = f.(data_name_list{i}).(joint_name_list{j}).R_w_com;
    end
    T_j_com{5} = f.(data_name_list{i}).(joint_name_list{j}).T_j_com;
    R_w_com{5} = f.(data_name_list{i}).(joint_name_list{j}).R_w_com;
end

%% Solve
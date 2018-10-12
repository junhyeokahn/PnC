close all; clc; clear;

%% Read File and Construct Matrix
fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions/yaml';
addpath(fn_path)
f = YAML.read('/Users/junhyeokahn/Repository/PnC/Config/FixedDraco/SysID/MASS.yaml');
% 1. Selection Matrix
S = [];
% 2. for each joint
t_j_c = {}; % (num data, num

%% Data Pre-processing
% dm = sym('m', [nj,1]);
% dr = sym('r', [nj,3]);

%% Generate Data
% Ax = y, where A = [A1; A2; ...; A
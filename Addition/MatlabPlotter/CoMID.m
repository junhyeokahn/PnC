close all; clc; clear;

%% Read File
addpath('/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions/yaml')
addpath('/Users/junhyeokahn/Repository/PnC/Addition/RoboticsUtils/MATLAB/')
f = YAML.read('/Users/junhyeokahn/Repository/PnC/Config/FixedDraco/SysID/MASS.yaml');

data_name_list = fieldnames(f);
joint_name_list = fieldnames(f.data0);
num_data = length(data_name_list);
num_joint = length(joint_name_list);
grav_vec = [0,0,0,0,0,-9.81]';
mass = [2.962, 0.408, 5.0, 3.1, 0.4];
xyz = [0, -0.07, 0.05, 0, -0.022, -0.02, 0.0009, 0.0066, -0.24, -0.0161, -0.0168, -0.275, 0.0108, 0, -0.0443];
%% Selection Matrix (get this from urdf)
axis{1} = [0, 0, 1, 0, 0, 0];
axis{2} = [1, 0, 0, 0, 0, 0];
axis{3} = [0, 1, 0, 0, 0, 0];
axis{4} = [0, 1, 0, 0, 0, 0];
axis{5} = [0, 1, 0, 0, 0, 0];
aug_S = zeros(num_joint, 6*num_joint);
for i = 1 : num_joint
    aug_S(i, 6*(i-1)+1:6*i) = axis{i};
end
 
%% Decision Variable
dm = sym('m', [5, 1], 'real');
dr = sym('r', [3, 5], 'real');
dmr = sym('mr', [3, 5], 'real');

for i = 1 : num_joint
    if i == 1
        decision_variables = [dm(i);dmr(:,i)];
        subs_ = [dm(i)*dr(:,i)]';
        subs__ = [dmr(:, i)'];
    else
        decision_variables = [decision_variables;dm(i);dmr(:,i)];
        subs_ = [subs_, (dm(i)*dr(:,i))'];
        subs__ = [subs__, dmr(:, i)'];
    end
end

%% Data Pre-processing
for data_id = 1 : num_data
    display(data_id);
    % Read Data
    for joint_id = 1 : 4
        T_j_nextj{joint_id} = f.(data_name_list{data_id}).(joint_name_list{joint_id}).T_j_j;
        T_j_com{joint_id} = f.(data_name_list{data_id}).(joint_name_list{joint_id}).T_j_com;
        T_j_com_sym{joint_id} = vpa([T_j_com{joint_id}(1:3, 1:3), dr(1:3, joint_id);0, 0, 0, 1], 2);
        R_w_com{joint_id} = f.(data_name_list{data_id}).(joint_name_list{joint_id}).R_w_com;
        torque_sensed{joint_id} = f.(data_name_list{data_id}).(joint_name_list{joint_id}).torque;
    end
    T_j_com{5} = f.(data_name_list{data_id}).(joint_name_list{5}).T_j_com;
    T_j_com_sym{5} = vpa([T_j_com{5}(1:3, 1:3), dr(1:3, 5);0, 0, 0, 1], 2);
    R_w_com{5} = f.(data_name_list{data_id}).(joint_name_list{5}).R_w_com;
    torque_sensed{5} = f.(data_name_list{data_id}).(joint_name_list{5}).torque;
    
    % Costruct Matrix
    for joint_id = 1 : num_joint
        for com_id = joint_id : num_joint
            if joint_id == com_id
                T_j_rec_com{joint_id, com_id} = T_j_com{com_id};
                T_j_rec_com_sym{joint_id, com_id} = T_j_com_sym{com_id};
                T_j_recj = eye(4);
            else
                T_j_recj = T_j_recj * T_j_nextj{com_id-1};
                T_j_rec_com{joint_id, com_id} = T_j_recj*T_j_com{com_id};
                T_j_rec_com_sym{joint_id, com_id} = T_j_recj * T_j_com_sym{com_id};
            end
            aug_Ad_list_sym{joint_id, com_id} = vpa(Adjoint(inv(T_j_rec_com_sym{joint_id, com_id}))', 2);
            aug_Ad_list{joint_id, com_id} = vpa(Adjoint(inv(T_j_rec_com{joint_id, com_id}))', 2);
        end
        aug_R_w_com = eye(6);
        aug_R_w_com(1:3, 1:3) = R_w_com{joint_id}';
        aug_R_w_com(4:6, 4:6) = R_w_com{joint_id}';
        if joint_id == 1
            aug_Wr = vpa(aug_R_w_com * mass(joint_id)*grav_vec, 2);
            aug_Wr_sym = vpa(aug_R_w_com * dm(joint_id)*grav_vec, 2);
        else
            aug_Wr = [aug_Wr; vpa(aug_R_w_com * mass(joint_id)*grav_vec, 2)];
            aug_Wr_sym = [aug_Wr_sym; vpa(aug_R_w_com * dm(joint_id)*grav_vec, 2)];
        end
    end
    
    for row_id = 1 : num_joint
        for col_id = 1 : num_joint
            if row_id == 1 && col_id ==1
                row_sym = aug_Ad_list_sym{row_id, col_id};
                row = aug_Ad_list{row_id, col_id};
            elseif row_id > col_id
                if col_id == 1
                    row_sym = zeros(6, 6);
                    row = zeros(6, 6);
                else
                    row_sym = [row_sym, eye(6, 6)];
                    row = [row, eye(6, 6)];
                end
            else
                row_sym = [row_sym, aug_Ad_list_sym{row_id, col_id}];
                row = [row, aug_Ad_list{row_id, col_id}];
            end
        end
        if row_id == 1
            aug_Ad_sym = vpa(row_sym,2);
            aug_Ad = vpa(row,2);
        else
            aug_Ad_sym = [vpa(aug_Ad_sym,2); row_sym];
            aug_Ad = [vpa(aug_Ad,2); row];
        end
    end
    
    % Extract Regression Matrix
    lhs = aug_S*aug_Ad*aug_Wr;
    lhs_sym = aug_S*aug_Ad_sym*aug_Wr_sym;
    % A <-- (5, 4*num_joint)
    % x <-- (4*num_joint)
    % b <-- (5)
    for joint_id = 1 : num_joint
        expanded = expand(lhs_sym(joint_id));
        change_of_var = subs(expanded, subs_, subs__);
        if joint_id == 1
            A = equationsToMatrix(change_of_var, decision_variables);
            b = -torque_sensed{joint_id};
        else
            A = [A; equationsToMatrix(change_of_var, decision_variables)];
            b = [b; -torque_sensed{joint_id}];
        end
    end
    
    if data_id == 1
        aug_A = A;
        aug_b = b;
    else
        aug_A = [aug_A; A];
        aug_b = [aug_b; b];
    end
    % Analytic Comparision
    for i = 1 : num_joint
        if i == 1
            dec=[mass(1); mass(1)*xyz(1:3)'];
        else
            dec=[dec; mass(i); mass(i)*xyz(3*i-2:3*i)'];
        end
    end
    A*dec - b
%     lhs-b
end

%% Solve Ax = b

aug_A_double = double(aug_A);
aug_b_double = double(aug_b);

d = 0.0001;
sol = inv(aug_A_double'*aug_A_double+d*eye(20, 20))*aug_A_double'*aug_b_double
%sol = inv(aug_A'*aug_A+d*eye(20, 20))*aug_A'*aug_b
true_cost = sum((aug_A*dec - aug_b).^2)
sol_cost = sum((aug_A*sol - aug_b).^2)


% main_p1_ilqc: Main script for Problem 2.1 ILQC controller design.
%
% --
% Control for Robotics
% Assignment 2
%
% --
% Technical University of Munich
% Learning Systems and Robotics Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@tum.de
% Lukas Brunke: lukas.brunke@tum.de
%
% --
% Revision history
% [20.01.31, SZ]    first version

clear all;
close all;

%% General
% add subdirectories
addpath(genpath(pwd));

% add task
task_ilqc = task_design();
N = length(task_ilqc.start_time:task_ilqc.dt:task_ilqc.end_time);

% add model
const_vel = 1; % assume constant forward speed
model = generate_model(const_vel);

% save directory
save_dir = './results/';

% initialize controller
load(strcat(save_dir, 'lqr_controller'));
controller_ilqc = controller_lqr;

% flags
plot_on = true;
save_on = true;

%% [Problem 2.1 (j)] Iterative Linear Quadratic Controller
% =========================== [TODO] ILQC Design ==========================
% Design an ILQC controller based on the linearized dynamics and
% quadratized costs. The cost function of the problem is specified in
% 'task_ilqc.cost' via the method 'task_design()'
%

for i = 1:1:task_ilqc.max_iteration
    sim_out_it = mobile_robot_sim(model,task_ilqc,controller_ilqc); %Initial Forward Pass
    [g_n_,q_n,Q_n] = terminal_cost_quad(task_ilqc.cost.params.Q_t, task_ilqc.goal_x, sim_out_it.x(:,end));
    s_k_ = g_n_;
    s_k = q_n;
    S_k = Q_n;

    for k = 1:1:N-2 %Backward Pass
        [A_k,B_k] = mobile_robot_lin(sim_out_it.x(:,end-k),sim_out_it.u(:,end-k),task_ilqc.dt,model.param.const_vel);
        [g_k_,q_k,Q_k,r_k,R_k,P_k] = stage_cost_quad(task_ilqc.cost.params.Q_s,task_ilqc.cost.params.R_s,task_ilqc.goal_x,task_ilqc.dt,sim_out_it.x(:,end-k),sim_out_it.u(:,end-k));
        [theta_kff,theta_kfb,s_k_,s_k,S_k] = update_policy(A_k,B_k,g_k_,q_k,Q_k,r_k,R_k,P_k,s_k_,s_k,S_k,sim_out_it.x(:,end-k),sim_out_it.u(:,end-k));
        controller_ilqc(:,(end-k)) = [theta_kff theta_kfb]';
    end
end

%
% =========================================================================

%% Simulation
sim_out_ilqc = mobile_robot_sim(model, task_ilqc, controller_ilqc);
fprintf('--- ILQC ---\n\n');
fprintf('trajectory cost: %.2f \n', sim_out_ilqc.cost);
fprintf('target state [%.3f; %.3f]\n', task_ilqc.goal_x);
fprintf('reached state [%.3f; %.3f]\n', sim_out_ilqc.x(:,end));
fprintf('initial state [%.3f; %.3f]\n', task_ilqc.start_x);

%% Plots
if plot_on
    plot_results(sim_out_ilqc);
end

%% Save controller and simulation results
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'ilqc_controller'), 'controller_ilqc', ...
        'sim_out_ilqc', 'task_ilqc'); 
end
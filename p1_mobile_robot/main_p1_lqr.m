% main_p1_lqr: Main script for Problem 2.1 LQR controller design.
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
clc;

%% General
% add subdirectories
addpath(genpath(pwd));

% define task
task_lqr = task_design();
N = length(task_lqr.start_time:task_lqr.dt:task_lqr.end_time);

% add model
const_vel = 1; % desired forward speed
model = generate_model(const_vel);

% initialize controller
controller_lqr = zeros(3, N-1);

% save directory
save_dir = './results/';

% flags
plot_on = true;
save_on = true;

%% [Problem 2.1 (c)] LQR Controller
% =========================== [TODO] LQR Design ===========================
% Design an LQR controller based on the linearization of the system about
% an equilibrium point (x_eq, u_eq). The cost function of the problem is
% specified in 'task_lqr.cost' via the method 'task_design()'.
%
A = [0 model.param.const_vel; 0 0];
B = [0;1];

[K,S,P] = lqr(A,B,task_lqr.cost.params.Q_s,task_lqr.cost.params.R_s);

theta_fb = -K;
theta_ff = K*task_lqr.goal_x;
theta = [theta_ff theta_fb]';
controller_lqr = repmat(theta, 1, N-1);

% 
% =========================================================================

%% Simulation
sim_out_lqr = mobile_robot_sim(model, task_lqr, controller_lqr);
fprintf('--- LQR ---\n\n');
fprintf('trajectory cost: %.2f\n', sim_out_lqr.cost);
fprintf('target state [%.3f; %.3f]\n', task_lqr.goal_x);
fprintf('reached state [%.3f; %.3f]\n', sim_out_lqr.x(:,end));
fprintf('initial state [%.3f; %.3f]\n', task_lqr.start_x);

%% Plots
if plot_on
    plot_results(sim_out_lqr);
end

%% Save controller and simulation results
if save_on
    if ~exist(save_dir, 'dir')
       mkdir(save_dir); 
    end
    
    % save controller and simulation results
	save(strcat(save_dir, 'lqr_controller'), 'controller_lqr', ...
        'sim_out_lqr', 'task_lqr'); 
end
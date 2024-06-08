% LQR_Design: Implementation of the LQR controller.
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
% This script is adapted from the course on Optimal & Learning Control for
% Autonomous Robots at the Swiss Federal Institute of Technology in Zurich
% (ETH Zurich). Course Instructor: Jonas Buchli. Course Webpage:
% http://www.adrlab.org/doku.php/adrl:education:lecture:fs2015
%
% --
% Revision history
% [20.01.31]    first version

function [LQR_Controller, Cost] = LQR_Design(Model,Task)
% LQR_DESIGN Creates an LQR controller to execute the Task
% Controller: Controller structure
%             u(x) = theta([t]) ' * BaseFnc(x)
%             .theta:   Parameter matrix (each column is associated with 
%                       one control input)
%             .BaseFnc
%             .time

%% Initializations
state_dim = 12;
input_dim = 4;
K = zeros(input_dim, state_dim);
theta = init_theta();
theta_k = theta;

%% [Problem 2.2 (a)] Find optimal feedback gains according to an LQR controller
% Make use of the elements in Task.cost for linearization points and cost
% functions.

% =========================================================================
% [Todo] Define the state input around which the system dynamics should be
% linearized assuming that most of the time the quadrotor flies similar to
% hovering (Fz != 0)
%
% x_lin = ...;
% u_lin = ...;
x_lin = Task.cost.x_eq;
u_lin = Task.cost.u_eq;

% [Todo] The linearized system dynamics matrices A_lin B_lin describe the
% dynamics of the system around the equilibrium state. Use Model.Alin{1}
% and Model.Blin{1}.
%
% A_lin = ...;
% B_lin = ...;
A_lin = Model.Alin{1}(x_lin,u_lin,Model.param.syspar_vec);
B_lin = Model.Blin{1}(x_lin,u_lin,Model.param.syspar_vec);

% [Todo] Compute optimal LQR gain (see command 'lqr')
% Quadratic cost defined as Task.cost.Q_lqr,Task.cost.R_lqr
% 
% K = ...;
[K,S,P] = lqr(A_lin,B_lin,Task.cost.Q_lqr,Task.cost.R_lqr);
% =========================================================================

%% Design the actual controller using the optimal feedback gains K
% The goal of this task is to correctly fill the matrix theta.
LQR_Controller.BaseFnc = @Augment_Base;
LQR_Controller.time    = Task.start_time:Task.dt:(Task.goal_time-Task.dt);

% The quadrotor controller produces inputs u from a combination of 
% feedforward uff and feedback elements as follows:
%   u = [Fz, Mx, My, Mz]' = uff + K'(x_ref - x)
%                         = uff + K'x_ref - K'x
%                         = [uff + K'x_ref, -K' ]' * [1,x']'
%                         =        theta'          * BaseFnc

lqr_type = 'via_point';  % Choose 'goal_state' or 'via_point'
fprintf('LQR controller design type: %s \n', lqr_type);
Nt = ceil((Task.goal_time - Task.start_time)/Task.dt+1);

switch lqr_type
    case 'goal_state'
%% [Problem 2.2 (b)] Drive system to Task.goal_x with LQR Gain + feedforward
        % =================================================================
        % [Todo] Define equilibrium point x_eq correctly and use it to
        % generate feedforward torques (see handout Eqn.(12) and notes
        % above).
        %
        % x_ref = ...; % reference point the controller tries to reach
        % theta = ...; % dimensions (13 x 4)
        x_ref = Task.goal_x;
        u_ref = u_lin;
        theta_ff = [u_ref + K*x_ref];
        theta_fb = -K;
        theta = [theta_ff theta_fb]';

        % =================================================================
        
        % stack constant theta matrices for every time step Nt
        LQR_Controller.theta = repmat(theta,[1,1,Nt]);
        
    case 'via_point'  
%% [Problem 2.2 (c)] Drive system to Task.goal_x through via-point p1.
        t1 = Task.vp_time;
        p1 = Task.vp1; % steer towards this input until t1

        for t=1:Nt-1
           % ==============================================================
           % [Todo] x_ref at current time
           % ...
           %
           u_ref = u_lin;
           theta_fb = -K;

           if t*Task.dt < t1
            x_ref = p1;
            theta_ff = [u_ref + K*x_ref];

           else
            x_ref = Task.goal_x;
            theta_ff = [u_ref + K*x_ref];
           end
           % [Todo] time-varying theta matrices for every time step Nt 
           % (see handout Eqn.(12)) (size: 13 x 4)
           % theta_k = ...;
           theta_k = [theta_ff theta_fb]';
           % ==============================================================

           % save controller gains to struct
           LQR_Controller.theta(:,:,t) = theta_k;
        end
        
    otherwise
        error('Unknown lqr_type');
end

% Calculate cost of rollout
sim_out = Quad_Simulator(Model, Task, LQR_Controller);
Cost = Calculate_Cost(sim_out, Task);
end

function x_aug = Augment_Base(t,x)
% AUGMENT_BASE(t,x) Allows to incorporate feedforward and feedback
%
%   Reformulating the affine control law allows incorporating
%   feedforward term in a feedback-like fashion:
%   u = [Fz, Mx, My, Mz]' = uff + K(x - x_ref)
%                         = uff - K*x_ref + K*x
%                         = [uff - K*x_ref, K ] * [1,x']'
%                         =        theta'       * BaseFnc 

number_of_states = size(x,2);      % multiple states x can be augmented at once
x_aug = [ones(1,number_of_states); % augment row of ones
                    x           ];
end


function Cost = Calculate_Cost(sim_out, Task)
% Calculate_Cost(.) Asses the cost of a rollout sim_out 
%
%   Be sure to correctly define the equilibrium state (X_eq, U_eq) the 
%   controller is trying to reach.
X = sim_out.x;
U = sim_out.u;
Q = Task.cost.Q_lqr;
R = Task.cost.R_lqr;
X_eq = repmat(Task.cost.x_eq,1,size(X,2)-1); % equilibrium state LQR controller tries to reach
U_eq = repmat(Task.cost.u_eq,1,size(U,2));   % equilibrium input LQR controller tries to reach
Ex = X(:,1:end-1) - X_eq;                    % error in state
Eu = U - U_eq;                               % error in input

Cost = Task.dt * sum(sum(Ex.*(Q*Ex),1) + sum(Eu.*(R*Eu),1));
end

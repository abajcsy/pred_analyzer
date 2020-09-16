%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
%  See possible configuration files and create new ones in /matlab/config/
% params = mdpHuman3DSimpleEnv();
params = carHuman4DDrivingEnv();

%% Get dynamical system and controls.
dynSys = params.schemeData.dynSys;
% controls{1} = STOP
% controls{2} = DOWN
% controls{3} = UP
% controls{4} = LEFT
% controls{5} = LEFT_DOWN
% controls{6} = LEFT_UP
% controls{7} = RIGHT
% controls{8} = RIGHT_DOWN
% controls{9} = RIGHT_UP
controls_text = {'S', 'D', 'U', 'L', 'LD', 'LU', 'R', 'RD', 'RU'};
controls = dynSys.controls;
uThresh = dynSys.uThresh;

%% Setup init cond
z = params.initial_state;
num_dims = size(z);
num_dims = num_dims(2);

%% Computation Setup
max_iter = 30;
% belief_target = [0.85,0.95];
% belief_target = [0.05,0.15];

% Use this belief target set to run til max_iter.
belief_target = [0.96,0.95];

%% Maximize the Qfunction.
zmax_init = z;
zmax = zmax_init;
z_traj_max = [];
z_traj_max(1:num_dims, end+1) = cell2mat(zmax_init)';
i = 0;
while (~(belief_target(1) <= zmax{num_dims} && zmax{num_dims} <= belief_target(2)) && i < max_iter)
    max_b = -1;
    max_u = 0;
    for j=1:length(controls)
        u = controls{j};

        % get probability of this action under true goal.
        qfun = dynSys.q_funs{params.trueThetaIdx};
        pu = dynSys.pugivenxtheta(u, zmax, qfun);
        
        % == Uncomment this if you want to maximize the Q-function: == %
        %                   u* = argmax_u Q(x,u,g)
        if pu > max_b
            max_b = pu;
            max_u = u;
        end
        % ============================================================ %
        
        % == Uncomment this if you want to maximize the posterior: == %
        %                   u* = argmax_u b'(g | x, u)
        % propagate dynamics.
%         bnext = dynSys.belief_update(u,zmax);
%         
%         if bnext > max_b
%             max_b = bnext;
%             max_u = u;
%         end
        % ============================================================ %
    end
    % propagate dynamics.
    znext = dynSys.dynamics(zmax,max_u);
    z_traj_max(1:num_dims, end+1) = cell2mat(znext)';
    zmax = znext;
    i = i + 1;
end
z_traj_max_tau = 1:length(z_traj_max);

%% Plot!
compType = 'conf';
extraPltArgs.compType = compType;
extraPltArgs.uThresh = params.uThresh;
extraPltArgs.uMode = params.uMode;
extraPltArgs.saveFigs = false;
extraPltArgs.bdims = params.bdims;
if isfield(params.reward_info, 'obstacles')
    extraPltArgs.obstacles = params.reward_info.obstacles;
end

z_traj_max_tau_real = (z_traj_max_tau - ones(size(z_traj_max_tau)))*params.dt;


plotOptTraj(z_traj_max, z_traj_max_tau_real, params.thetas, params.trueThetaIdx, ...
    params.gmin, params.gmax, params.gnums, 0, extraPltArgs)

box on
grid off
set(gcf, 'position', [0,0,800,800]);
% title(strcat("Control Which Maximizes Next Belief: TTE=", num2str(z_traj_max_tau_real(end))));
title(strcat("Control Which Maximizes Q-function: TTE=", num2str(z_traj_max_tau_real(end))));

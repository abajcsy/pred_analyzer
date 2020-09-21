%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
%  See possible configuration files and create new ones in /matlab/config/
params = mdpHumanSGD3DSimpleEnv();

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
max_iter = 10;

% Use this belief target set to run til max_iter.
theta_target = 0.9;

%% Maximize the Qfunction.
zmax_init = {-2,-1, 0.1};
zmax = zmax_init;
z_traj_max = [];
z_traj_max(1:num_dims, end+1) = cell2mat(zmax_init)';
i = 0;
while (zmax{end} < theta_target)
    min_diff = 100000000;
    min_u = 0;
    for j=1:length(controls)
        u = controls{j};

        % get probability of this action under true goal.
        theta_n = dynSys.sgd(u, zmax);
        dist = norm(theta_target - theta_n);
        
        % == Uncomment this if you want to maximize the Q-function: == %
        %                   u* = argmax_u Q(x,u,g)
        if dist < min_diff
            min_diff = dist;
            min_u = u;
        end
        % ============================================================ %

    end
    % propagate dynamics.
    znext = dynSys.dynamics(zmax,min_u);
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


% ====================================== %
hold on
pt = plot3(z_traj_max(1,:), z_traj_max(2,:), z_traj_max(3,:), '-o');
pt.LineWidth = 2;
pt.MarkerFaceColor = 'r';
pt.MarkerEdgeColor = 'r';
hold on
for oi = 1:length(params.reward_info.obstacles)
    obs_info = params.reward_info.obstacles{oi};
    obs_min = obs_info(1:2);

    x_min = obs_min(1);
    y_min = obs_min(2);
    p_min = 0;
    l = [obs_info(3), ...
        obs_info(4), ...
        1];
    plotcube(l,[x_min y_min p_min], .5, [0.3 0.3 0.3]);
end
xlim([-4,4])
ylim([-4,4])
% ====================================== %

box on
grid off
set(gcf, 'position', [0,0,800,800]);
title(strcat("Control Which Gets Desired theta: TTE=", num2str(z_traj_max_tau_real(end))));

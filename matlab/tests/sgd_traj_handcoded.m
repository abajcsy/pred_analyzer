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

%% plot?
% dynSys.plot_opt_policy(1);
% dynSys.plot_opt_policy(3);

%% Setup init cond
z = params.initial_state;
num_dims = size(z);
num_dims = num_dims(2);

%% Computation Setup
max_iter = 10;

% Use this belief target set to run til max_iter.
theta_target = 0.5;
zmax_init = {-3,-1, 0.5};

%% SEE WHAT HAPPENS TO THETA AFTER EACH ACTION.
figure
for ui=1:length(dynSys.controls)
    u = dynSys.controls{ui};
    [theta_n, dQ] = dynSys.sgd(u, zmax_init);
    znext = dynSys.physical_dynamics(zmax_init,u);
    
    hold on
    sc = scatter3(zmax_init{1}, zmax_init{2}, zmax_init{3}, 'k');
    sc.MarkerFaceColor = 'k';
    sn = scatter3(znext{1}, znext{2}, theta_n, 'r');
    sn.MarkerFaceColor = [0.8,0,0];
    quiver3(zmax_init{1},zmax_init{2},zmax_init{3}, ...
        znext{1}-zmax_init{1},znext{2}-zmax_init{2},theta_n-zmax_init{3},...
        'Color', 'r', ...
        'linewidth', 1);  
    grid on
end
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
co = contour(params.reward_info.g.xs{1}, params.reward_info.g.xs{2}, ...
    dynSys.obs_feature, [0,0], 'color', 'b');
cg = contour(params.reward_info.g.xs{1}, params.reward_info.g.xs{2}, ...
    dynSys.goal_feature, [0,0], 'color', 'r');
set(gcf, 'color', 'w')
view(-54, 30);

%% hard-coded control sequence. 
% stop = 1
% u_seq = {3,3,3,3,3,3}; % up
% u_seq = {4,4,4,4,4,4}; % left
% u_seq = {6,6,6,6,6,6}; % left up
% u_seq = {7,7,7,7}; % right
 u_seq = {9,9,9,9,9,9}; % right up

%% Maximize the Qfunction.
zmax = zmax_init;
z_traj_max = [];
z_traj_max(1:num_dims, end+1) = cell2mat(zmax_init)';
i = 1;
while (i < length(u_seq))
    
    % apply GD upon seeing this control.
    uidx = u_seq{i};
    u = dynSys.controls{uidx};
    [theta_n, dQ] = dynSys.sgd(u, zmax);
        
    % propagate dynamics.
    znext = dynSys.dynamics(zmax,u);
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
figure
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
title(strcat("Hand-coded SGD traj: TTE=", num2str(z_traj_max_tau_real(end))));

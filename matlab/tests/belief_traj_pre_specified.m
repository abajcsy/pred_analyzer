%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
%  See possible configuration files and create new ones in /matlab/config/
params = mdpHuman3DSimpleEnv();

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

%% Maximize the Qfunction.
zmax_init = z;
zmax = zmax_init;
z_traj_max = [];
z_traj_max(1:3, end+1) = cell2mat(zmax_init)';
while zmax{3} < 0.85
    max_b = -1;
    max_u = 0;
    for i=1:length(controls)
        u = controls{i};

        % get probability of this action under true goal.
        qfun = dynSys.q_funs{params.trueThetaIdx};
        pu = dynSys.pugivenxtheta(u, zmax, qfun);
        
%         if pu > max_b
%             max_b = pu;
%             max_u = u;
%         end

        % propagate dynamics.
        bnext = dynSys.belief_update(u,zmax);
        
        if bnext > max_b
            max_b = bnext;
            max_u = u;
        end
    end
    % propagate dynamics.
    znext = dynSys.dynamics(zmax,max_u);
    z_traj_max(1:3, end+1) = cell2mat(znext)';
    zmax = znext;
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
title(strcat("Control Which Maximizes Next Belief: TTE=", num2str(z_traj_max_tau_real(end))));

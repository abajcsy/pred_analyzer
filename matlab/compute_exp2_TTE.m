%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
%  See possible configuration files and create new ones in /matlab/config/

% === Setups where joint state includes belief. === %
params = exp2_conf_reachability();

%% Sanity check -- Plot the opt control policies?
% params.dyn_sys.plot_opt_policy(1);
% params.dyn_sys.plot_opt_policy(2);

%% Plot optimal control policy starting from initial condition.
% params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 1);
% params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 2);

%% Solve for the discrete-time value function!
[value_funs, tauOut, extraOuts] = ...
    DiscTimeHJIPDE_solve(params.initial_value_fun, ...
                         params.tau, ...
                         params.schemeData, ...
                         params.minWith, ...
                         params.extraArgs);

%% Find and plot optimal control sequence (if reachable by computed BRS)
fprintf("Computing opt traj...\n");
[traj, traj_tau, ctrls] = computeOptTraj(params.initial_state, ....
                                  params.g, ...
                                  value_funs, ...
                                  tauOut, ...
                                  params.dyn_sys, ...
                                  params.uMode, ...
                                  params.extraArgsCtrl);

fprintf('===> TTE: %f\n', traj_tau(end)*params.dt);
% ====================================== %
hold on
pt = plot3(traj(1,:), traj(2,:), traj(3,:), '-o');
pt.LineWidth = 2;
pt.Color = 'k';
pt.MarkerFaceColor = 'b';
pt.MarkerEdgeColor = 'b';
hold on
if ~isa(params.reward_info.obstacles, 'Grid')
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
end
xlim([params.gmin(1),params.gmax(1)])
ylim([params.gmin(2),params.gmax(2)])
box on
grid on
% ====================================== %
%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
%  See possible configuration files and create new ones in /matlab/config/
% params = mdpHuman3DSimpleEnv();
% params = mdpHuman3DDrivingEnv();
params = carHuman4DDrivingEnv();

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
[traj, traj_tau] = computeOptTraj(params.initial_state, ....
                                  params.g, ...
                                  value_funs, ...
                                  tauOut, ...
                                  params.dyn_sys, ...
                                  params.uMode, ...
                                  params.extraArgsCtrl);

%% Plot optimal trajectory and value functions
if params.plot
    compType = 'conf';
    extraPltArgs.compType = compType;
    extraPltArgs.uThresh = params.uThresh;
    extraPltArgs.uMode = params.uMode;
    extraPltArgs.saveFigs = false;
    extraPltArgs.bdims = params.bdims;
    if isfield(params.reward_info, 'obstacles')
        extraPltArgs.obstacles = params.reward_info.obstacles;
    end
    goalSetRad = 0;
    
    % IF USING THE MDP HUMAN:
    %   Convert mdp timesteps to real time.
    traj_tau_real = (traj_tau - ones(size(traj_tau)))*params.dt;
    
    % Plot the optimal traj
    plotOptTraj(traj, traj_tau_real, ...
                params.thetas, params.trueThetaIdx, ...
                params.gmin, params.gmax, params.gnums, ...
                goalSetRad, extraPltArgs);
    
    % Plot the BRS.
%     visBRSVideo(params.g, ...
%                 value_funs, ...
%                 params.initial_state, ...
%                 tauOut, ...
%                 extraOuts.stoptau, ...
%                 params.dt);
%     visBRSSubplots(g, value_funs, initial_state, tau);
end

% save(strcat('e2_b50_finer_grid_brt_',uMode,'_uthresh',num2str(uThresh),'.mat'), 'g', 'gmin', 'gmax', 'gnums', ...
%     'value_funs', 'tau', 'traj', 'traj_tau', 'uMode', 'initial_value_fun', ...
%     'schemeData', 'minWith', 'extraArgs', 'thetas', 'trueThetaIdx', 'extraPltArgs');

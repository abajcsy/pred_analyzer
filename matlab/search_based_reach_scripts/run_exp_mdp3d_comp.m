%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
%  See possible configuration files and create new ones in /matlab/config/

% === Setups where joint state includes belief. === %
params = exp_mdp3d_comp();

%% Sanity check -- Plot the opt control policies?
% params.dyn_sys.plot_opt_policy(1);
% params.dyn_sys.plot_opt_policy(2);

%% Plot optimal control policy starting from initial condition.
%params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 1);
%params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 2);

%% Plot the likely controls at state for true theta. 
% params.dyn_sys.plot_pu_given_x({1,1,0.5}, params.thetas, 2);

%% Solve for the discrete-time value function!
% [value_funs, tauOut, extraOuts] = ...
%     DiscTimeHJIPDE_solve(params.initial_value_fun, ...
%                          params.tau, ...
%                          params.schemeData, ...
%                          params.minWith, ...
%                          params.extraArgs);

[value_funs, tauOut, extraOuts] = ...
    DiscTimeHJIPDE_solve_DEBUG(params.initial_value_fun, ...
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

if isfield(params, "save") && params.save
    repo = what('pred_analyzer');
    data_path = strcat(repo.path, '/matlab/data/');
    save(data_path+params.exp_type+"_"+params.uMode+"_uthresh_"+num2str(params.uThresh)+"_theta_"+num2str(params.trueThetaIdx)+".mat", ...
        "traj", "traj_tau", "ctrls", "params", "value_funs", "extraOuts");
end

% print out benchmarking stats
fprintf('========== TIME BENCHMARKS ========= \n');
fprintf('Time to pre-compute Q-FUNCTIONS (s): %f\n', params.dyn_sys.precomp_q_t);
fprintf('Time to pre-compute LIKELY CTRLS (s): %f\n', extraOuts.precomp_ctrls_t);
fprintf('Time to pre-compute DYNAMICS (s): %f\n', extraOuts.precomp_dyns_t);
fprintf('Time to DP computation time (s): %f\n', extraOuts.dp_comp_t);
fprintf('=================================== \n');

%[gOut, dataOut] = proj(params.g, value_funs{1}, [0,0,1], 'max');
%clf; visSetIm(gOut, dataOut)

% ====================================== %
% hold on
% pt = plot3(traj(1,:), traj(2,:), traj(3,:), '-o');
% pt.LineWidth = 2;
% pt.Color = 'k';
% pt.MarkerFaceColor = 'b';
% pt.MarkerEdgeColor = 'b';
% hold on
% if ~isa(params.reward_info.obstacles, 'Grid')
%     for oi = 1:length(params.reward_info.obstacles)
%         obs_info = params.reward_info.obstacles{oi};
%         obs_min = obs_info(1:2);
% 
%         x_min = obs_min(1);
%         y_min = obs_min(2);
%         p_min = 0;
%         l = [obs_info(3), ...
%             obs_info(4), ...
%             1];
%         plotcube(l,[x_min y_min p_min], .5, [0.3 0.3 0.3]);
%     end
% end
% xlim([params.gmin(1),params.gmax(1)])
% ylim([params.gmin(2),params.gmax(2)])
% box on
% grid on
% ====================================== %
   

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

end

figure
for t=30:-1:1
    visSetIm(params.g, value_funs{t});
end


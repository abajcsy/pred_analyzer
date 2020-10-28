%% File to compute reachable sets and optimal control via discrete-time HJ
clear all
clf
close all

%% Load all the parameters for this computation!
%  See possible configuration files and create new ones in /matlab/config/

% === Setups where joint state includes belief. === %
% params = mdpHuman3DSimpleEnv();
% params = mdpHuman3DDrivingEnv();
% params = carHuman4DDrivingEnv();
% params = carHuman4DDrivingFullEnv();

% params = mdpHumanConfidence4DSimpleEnv();
% params = mdpHumanConfidence3DSimpleEnv();

% params = exp1_legibility();
% params = exp2_confidence();
params = exp4_gradient();

% === Setups where joint state includes direct parameter vals. === %
% params = mdpHumanSGD3DSimpleEnv();

%% Sanity check -- Plot the opt control policies?
% params.dyn_sys.plot_opt_policy(1);
% params.dyn_sys.plot_opt_policy(2);
% params.dyn_sys.plot_opt_policy(3);

% params.dyn_sys.plot_opt_policy_from_x0({-3.40741,0.296296},0.25);

%% Plot optimal control policy starting from initial condition.
% params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 1);
% params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 2);
% params.dyn_sys.plot_opt_policy_from_x0(params.initial_state, 3);

%% Solve for the discrete-time value function!
[value_funs, tauOut, extraOuts] = ...
    DiscTimeHJIPDE_solve(params.initial_value_fun, ...
                         params.tau, ...
                         params.schemeData, ...
                         params.minWith, ...
                         params.extraArgs);
                     
% % Plot the BRS.
% visBRSVideo(params.g, ...
%             value_funs, ...
%             params.initial_state, ...
%             tauOut, ...
%             extraOuts.stoptau, ...
%             params.dt);

%% Visualize Projection of Value Funs
row_len = 5;
figure;
for i=1:numel(value_funs)
    h = subplot(floor(numel(value_funs)/row_len) + 1,row_len,i);
    axis square
    [gOut, dataOut] = proj(params.g, value_funs{numel(value_funs)-numel(value_funs)+i}, [1,1,0], 'min');
    visSetIm(gOut, dataOut);
    xlim([0,1]);
    ylim([-0.4,0.4]);
    t = title(['t=',num2str((i-1)),' s'], 'Interpreter', 'Latex');
    t.FontSize = 18;
    grid on;
end

row_len = 5;
figure;
for i=1:numel(value_funs)
    hold on;
%     ph = pcolor(params.reward_info.obstacles.g{1}, ...
%                 params.reward_info.obstacles.g{2}, ...
%                 params.reward_info.obstacles.data);
%     set(ph, 'EdgeColor', 'none');
    h = subplot(floor(numel(value_funs)/row_len) + 1,row_len,i);
    axis square
    [gOut, dataOut] = proj(params.g, value_funs{numel(value_funs)-numel(value_funs)+i}, [0,0,1], 'min');
    visSetIm(gOut, dataOut);
    xlim([-4,4]);
    ylim([-4,4]);
    t = title(['t=',num2str((i-1)),' s'], 'Interpreter', 'Latex');
    t.FontSize = 18;
    grid on;
    hold off;
end

% [gOut, dataOut] = proj(params.g, value_funs{1}, [1,1,0], 'min');
% dataOut

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
    save("data/"+params.exp_type+"_"+params.uMode+"_uthresh_"+num2str(params.uThresh)+"_theta_"+num2str(params.trueThetaIdx)+".mat", ...
        "traj", "traj_tau", "ctrls", "params", "value_funs");
end

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
    if isfield(params,"theta")
        plotOptTraj(traj, traj_tau_real, ...
                    {params.theta}, 1, ...
                    params.gmin, params.gmax, params.gnums, ...
                    goalSetRad, extraPltArgs);
    else 
        plotOptTraj(traj, traj_tau_real, ...
                params.thetas, params.trueThetaIdx, ...
                params.gmin, params.gmax, params.gnums, ...
                goalSetRad, extraPltArgs);
    end
    
    % Plot control values
%     plotControlValues(ctrls, params.dyn_sys);
    
    % Plot the BRS.
%     visBRSVideo(params.g, ...
%                 value_funs, ...
%                 params.initial_state, ...
%                 tauOut, ...
%                 extraOuts.stoptau, ...
%                 params.dt);
%     visBRSSubplots(params.g, value_funs, params.initial_state, traj_tau);
end

% save(strcat('e2_b50_finer_grid_brt_',uMode,'_uthresh',num2str(uThresh),'.mat'), 'g', 'gmin', 'gmax', 'gnums', ...
%     'value_funs', 'tau', 'traj', 'traj_tau', 'uMode', 'initial_value_fun', ...
%     'schemeData', 'minWith', 'extraArgs', 'thetas', 'trueThetaIdx', 'extraPltArgs');

function plotControlValues(ctrls,dynSys)
    vels = [];
    ang_vels = [];
    for i=1:numel(ctrls)
        ctrl = dynSys.get_ctrl(ctrls{i});
        vels = [vels ctrl(1)];
        ang_vels = [ang_vels ctrl(2)];
    end

    figure
    plot(vels)
    xlabel("time")
    ylabel("velocity")

    figure
    plot(ang_vels)
    xlabel("time")
    ylabel("angular velocity")
end

function valid = check_traj(traj, obs)
    [num_states, t_len] = size(traj);
    c = cell(1, num_states);
    valid = 1;
    for i=1:t_len
        s = traj(:,i);
        c{1} = s(1);
        c{2} = s(2);
        c{3} = s(3);
        if obs.GetDataAtReal(c) == 1
            valid = 0;
            break;
        end
    end
end

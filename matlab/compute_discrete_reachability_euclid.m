%% File to compute reachable sets and optimal control via discrete-time HJ

clear all
clf
close all

%% Grid setup
gmin = [-4, -4, 0];
gmax = [4, 4, 1];
gnums = [30, 30, 30];
g = createGrid(gmin, gmax, gnums);

%% Target Set Setup
tol = 0.2;
centerPgoal1 = 0.9;
xyoffset = 0.1;
center = [0; 0; centerPgoal1];
widths = [(gmax(1) - gmin(1)) - xyoffset; ...
          (gmax(2) - gmin(2)) - xyoffset; 
          tol];
% tol = 0.2;
% centerPgoal1 = 0.1;
% xyoffset = 0.1;
% center = [-2; 2; centerPgoal1];
% widths = [1; ...
%           1; 
%           tol];
initial_value_fun = shapeRectangleByCenter(g, center, widths);

%% Time vector
t0 = 1;
num_timesteps = 25;
tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
uMode = "min"; % min or max
uThresh = 0.00; %0.13; %0.14; % 0.16;

%% Plotting?
plot = true;        % Visualize the BRS and the optimal trajectory?

%% Joint Dynamics Setup.
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;

% Initial state and dynamical system setup
initial_state = {-2,0,0.5}; %{0, 0, 0.1};

% MDP human.
gdisc = (gmax - gmin) ./ (gnums - 1);
dyn_sys = MDPHumanBelief3DEuclid(initial_state, thetas, trueThetaIdx, uThresh, gdisc);

% Discrete control angle human. 
% v = 1.0;
% n = 20;
% dyn_sys = HumanBelief3DEuclid(initial_state, thetas, trueThetaIdx, uThresh, dt, v, n);

%% Pack problem parameters
schemeData.grid = g;
schemeData.dynSys = dyn_sys;
schemeData.uMode = uMode;

%% Adding obstacles
existsObstacle = false;
% obs_center = [-1; 1; 0.5];
% obs_width = [1; ...
%           1; 
%           1.0];
% obstacle_fun = -1 .* shapeRectangleByCenter(g, obs_center, obs_width);
% extraArgs.obstacles = obstacle_fun;

%% Pack value function params
if existsObstacle
    initial_value_fun = max(initial_value_fun,obstacle_fun);
end

extraArgs.targets = initial_value_fun;
% extraArgs.stopInit = initial_state;

% 'none' or 'set' for backward reachable set (BRS)
% 'minVWithL' for backward reachable tube (BRT)
minWith = "minVWithL"; 

%% Optimal control params
extraArgsCtrl.interpolate = false;

%% Solve for the discrete-time value function!
[value_funs, tau, extraOuts] = ...
    DiscTimeHJIPDE_solve(initial_value_fun, tau, schemeData, minWith, extraArgs);

%% Find and plot optimal control sequence (if reachable by computed BRS)
[traj, traj_tau] = computeOptTraj(initial_state, g, value_funs, tau, dyn_sys, uMode, extraArgsCtrl);

%% Plot optimal trajectory and value functions
% if isfield(extraOuts, 'stoptau')
%     start_idx = extraOuts.stoptau;
% else
%     start_idx = 1;
% end
if plot
    compType = 'conf';
    extraPltArgs.compType = compType;
    extraPltArgs.uThresh = uThresh;
    extraPltArgs.uMode = uMode;
    extraPltArgs.saveFigs = false;
    if existsObstacle
        extraPltArgs.existsObstacle = existsObstacle;
        extraPltArgs.obs_center = obs_center;
        extraPltArgs.obs_width = obs_width;
        extraPltArgs.obstacles = obstacle_fun;
    end
    
    % Plot the optimal traj
    plotOptTraj(traj, traj_tau, thetas, trueThetaIdx, ...
        gmin, gmax, gnums, 0, value_funs, extraPltArgs);
    
    % Plot the BRS.
%     visBRSVideo(g, value_funs, initial_state, tau);
%     visBRSSubplots(g, value_funs, initial_state, tau);
end

save(strcat('e2_b50_finer_grid_brt_',uMode,'_uthresh',num2str(uThresh),'.mat'), 'g', 'gmin', 'gmax', 'gnums', ...
    'value_funs', 'tau', 'traj', 'traj_tau', 'uMode', 'initial_value_fun', ...
    'schemeData', 'minWith', 'extraArgs', 'thetas', 'trueThetaIdx', 'extraPltArgs');

%% Helper functions
% TODO: Make x_ind, b_ind depend on real values not indices
function value_fun = construct_value_fun_fmm(x_ind, y_ind, b_ind, gmin, gmax, gnums)
    grid = createGrid(gmin, gmax, gnums);
    target_set = ones(size(grid.xs{1}));
    target_set(x_ind(1):x_ind(2),y_ind(1):y_ind(2),b_ind(1):b_ind(2)) = -1;
    value_fun = compute_fmm_map(grid, target_set);
end

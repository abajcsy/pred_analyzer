%% File to compute reachable sets and optimal control via discrete-time HJ

clear all
clf
close all

%% Grid setup
gmin = [-4, -4, 0];
gmax = [4, 4, 1];
gnums = [20, 20, 20];
g = createGrid(gmin, gmax, gnums);

%% Target Set Setup
tol = 0.1;
centerPgoal1 = 0.9;
xyoffset = 0.1;
center = [0; 0; centerPgoal1];
widths = [(gmax(1) - gmin(1)) - xyoffset; ...
          (gmax(2) - gmin(2)) - xyoffset; 
          tol];
initial_value_fun = shapeRectangleByCenter(g, center, widths);

%% Time vector
t0 = 1;
num_timesteps = 9;
tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
uMode = "min"; % min or max
uThresh = 0.00;%0.14; % 0.16;

%% Plotting?
plot = true;        % Visualize the BRS and the optimal trajectory?

%% Joint Dynamics Setup.
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;

% Initial state and dynamical system setup
initial_state = {-1.474, -1.053, 0.1355};

% MDP human.
gdisc = (gmax - gmin) ./ (gnums - 1);
dyn_sys = MDPHumanBelief2D(initial_state, thetas, trueThetaIdx, uThresh, gdisc);

% Discrete control angle human. 
% v = 1.0;
% n = 20;
% dyn_sys = HumanBelief2D(initial_state, thetas, trueThetaIdx, uThresh, dt, v, n);

%% Pack problem parameters
schemeData.grid = g;
schemeData.dynSys = dyn_sys;
schemeData.uMode = uMode;

%% Pack value function params
extraArgs.targets = initial_value_fun;
% extraArgs.stopInit = initial_state;

% 'none' or 'set' for backward reachable set (BRS)
% 'minVWithL' for backward reachable tube (BRT)
minWith = "minVWithL"; 

%% Solve for the discrete-time value function!
[value_funs, tau, extraOuts] = ...
    DiscTimeHJIPDE_solve(initial_value_fun, tau, schemeData, minWith, extraArgs);

%% Find and plot optimal control sequence (if reachable by computed BRS)
[traj, traj_tau] = computeOptTraj(initial_state, g, value_funs, tau, dyn_sys, uMode);
    
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
    
    % Plot the optimal traj
    plotOptTraj(traj, traj_tau, thetas, trueThetaIdx, ...
        gmin, gmax, gnums, 0, value_funs, extraPltArgs);
    
    % Plot the BRS.
%     visBRSVideo(g, value_funs, initial_state, tau);
%     visBRSSubplots(g, value_funs, initial_state, tau);
end

%% Helper functions
% TODO: Make x_ind, b_ind depend on real values not indices
function value_fun = construct_value_fun_fmm(x_ind, y_ind, b_ind, gmin, gmax, gnums)
    grid = createGrid(gmin, gmax, gnums);
    target_set = ones(size(grid.xs{1}));
    target_set(x_ind(1):x_ind(2),y_ind(1):y_ind(2),b_ind(1):b_ind(2)) = -1;
    value_fun = compute_fmm_map(grid, target_set);
end

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
tol = 0.2;
centerPgoal1 = 0.9;
xyoffset = 0.1;
center = [0; 0; centerPgoal1];
widths = [(gmax(1) - gmin(1)) - xyoffset; ...
          (gmax(2) - gmin(2)) - xyoffset; 
          tol];
initial_value_fun = shapeRectangleByCenter(g, center, widths);

%% Time vector
t0 = 1;
num_timesteps = 6;
tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
uMode = "min"; % min or max
uThresh = 0.15;%0.15;%0.13; %0.14; % 0.16;

%% Plotting?
plot = true;        % Visualize the BRS and the optimal trajectory?

%% Joint Dynamics Setup.
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;
gamma = 0.9;
eps = 0.01;

% Obstacles for Q-function
obs_center = [-1; 1];
obs_width = [1; ...
          1;];
      
goal_val = 1;
% obs_val = -0;
obs_val = -inf;

goal_rad = 0.5;
g_phys = createGrid([gmin(1), gmin(2)], [gmax(1), gmax(2)], [gnums(1), gnums(2)]);

v_inits = cell(1,numel(thetas));
for i=1:numel(v_inits)
    v_data = compute_v_data(g_phys, goal_val, thetas{i}, goal_rad, obs_val, obs_center, obs_width);
    v_grid = Grid([gmin(1), gmin(2)], [gmax(1), gmax(2)], [gnums(1), gnums(2)]);
    v_grid.SetData(v_data);
    v_inits{i} = v_grid;
end

% Initial state and dynamical system setup
initial_state = {0, 0, 0.1};

% MDP human.
gdisc = (gmax - gmin) ./ (gnums - 1);
dyn_sys = MDPHumanBelief2D_Q(initial_state, v_inits, trueThetaIdx, uThresh, gdisc, gamma, eps);

% Discrete control angle human. 
% v = 1.0;
% n = 20;
% dyn_sys = HumanBelief3DEuclid(initial_state, thetas, trueThetaIdx, uThresh, dt, v, n);

%% Pack problem parameters
schemeData.grid = g;
schemeData.dynSys = dyn_sys;
schemeData.uMode = uMode;

%% Plot obstacles
existsObstacle = true;
plotObstacle = true;
obs_center = [-1; 1; 0.5];
obs_width = [1; ...
          1; 
          1.0];
obstacle_fun = -1 .* shapeRectangleByCenter(g, obs_center, obs_width);

%% Add obstacles to reachability
if existsObstacle
    extraArgs.obstacles = obstacle_fun;
end


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
    if plotObstacle 
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

%% Helper functions
% TODO: Make x_ind, b_ind depend on real values not indices
function value_fun = construct_value_fun_fmm(x_ind, y_ind, b_ind, gmin, gmax, gnums)
    grid = createGrid(gmin, gmax, gnums);
    target_set = ones(size(grid.xs{1}));
    target_set(x_ind(1):x_ind(2),y_ind(1):y_ind(2),b_ind(1):b_ind(2)) = -1;
    value_fun = compute_fmm_map(grid, target_set);
end

% TODO: Change goal_fun to circle (using shapeSphere).
function v_data = compute_v_data(g, goal_value, goal_center, goal_rad, obs_value, obs_center, obs_width)
    goal_fun = shapeRectangleByCenter(g, ...
        [goal_center(1); goal_center(2)], [goal_rad; goal_rad]);
    
    goal_fun(goal_fun>=0) = 0;
    goal_fun(goal_fun<0) = goal_value;
    
    
    obs_fun = shapeRectangleByCenter(g, obs_center, obs_width);
    obs_fun(obs_fun<0) = obs_value;
    obs_fun(obs_fun>=0) = 0;
    
    v_data = goal_fun + obs_fun;
end
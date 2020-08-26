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
num_timesteps = 15;
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
obs_val = -inf;

grid = Grid([gmin(1), gmin(2), gmin(3)], [gmax(1), gmax(2), gmax(3)], [gnums(1), gnums(2), gnums(3)]);

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

existsObstacle = true;
obs_center = [-1; 1; 0.5];
obs_width = [1; ...
          1; 
          1.0];
obstacle_fun = -1 .* shapeRectangleByCenter(g, obs_center, obs_width);

%% Compute greedy trajectory
grid_vals = true;

if grid_vals
    state = grid.RealToCoords(initial_state);
else
    state = initial_state;
end

traj = nan(g.dim, 20);
traj_tau = [0];
traj(:,1) = cell2mat(state);
j = 1;
while state{3} < 0.8
    j = j + 1;
    belief_vals = zeros(1,numel(dyn_sys.controls));
    for i=1:numel(dyn_sys.controls)
        u_i = dyn_sys.controls{i};
        next_state_i = dyn_sys.dynamics(state,u_i);
        if grid_vals
            next_state_i = grid.RealToCoords(next_state_i);
        end
        prob = dyn_sys.pugivenxtheta(u_i,state,dyn_sys.q_funs{trueThetaIdx});
        likely = nan;
        if prob >= uThresh
            likely = 1;
        end
        belief_vals(i) = next_state_i{3} * likely;
    end
    [opt_belief,IX] = max(belief_vals,[],2);
    state = dyn_sys.dynamics(state,dyn_sys.controls{IX});
    if grid_vals
        state = grid.RealToCoords(state);
    end
    traj(:,j) = cell2mat(state);
    traj_tau = [traj_tau j-1];
end

traj = traj(:,1:j);

value_funs = cell(1, j);
for i=1:j
    value_funs{i} = initial_value_fun;
end

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
function params = exp4_gradient()

%% Grid setup
params.gmin = [-4, -4, 0];
params.gmax = [4, 4, 0.95];
params.gnums = [40, 40, 40];
params.g = createGrid(params.gmin, params.gmax, params.gnums);
params.bdims = 3; % dimension(s) which contain the belief

%% Belief Grid Setup
params.g_belief = createGrid(params.gmin(params.bdims), ...
                                params.gmax(params.bdims), ...
                                params.gnums(params.bdims));

%% Time vector
t0 = 1;
num_timesteps = 10;
params.tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
params.uMode = "min";       % min or max
params.uThresh = 0.0;       % threshold on P(u | x, g) -- e.g. 0.15;%0.14;%0.13;

%% Plotting?
params.plot = false;         % Visualize the BRS and the optimal trajectory?

%% Joint Dynamics Setup.
params.thetas = {0,0.5,0.98};%{0.5,0.75,0.98};%{0,0.5,0.98}; % discrete set of thetas to precompute Q-func for
params.trueTheta = 0.1; 
params.w1 = 1;

%% Pack Reward Info

% 2D Grid for computing Q-function.
g_phys = createGrid(params.gmin(1:2)', ...
                    params.gmax(1:2)', ...
                    params.gnums(1:2)');
params.reward_info.g = g_phys; 
params.reward_info.goal = [1; 2]; %[-2; 2]; 
params.reward_info.goalRad = 0.5; %1.3;

% Obstacles (based on interpolated occupancy grid) used in Q-function computation.
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/data/');
% map_name = 'map.png';
map_name = 'cluttered_map_doorway.png';
obs_data = imread(strcat(data_path, map_name));
n_phys = numel(params.gmin) - numel(params.bdims);
occupancy_map = get_obs_map(obs_data, ...
                            params.gmin(1:n_phys), ...
                            params.gmax(1:n_phys), ...
                            params.gnums(1:n_phys));
% Round occupancy map to 0 (free) or 1 (obs), turn it into made up of
% +1s (free) and -1s (obs), and compute signed distance function.
occupancy_map = occupancy_map.data;
occupancy_map(occupancy_map<=0) = 0;
occupancy_map = 1 - 2.*occupancy_map;
grid_phys = createGrid(params.gmin(1:n_phys), ...
                        params.gmax(1:n_phys), ...
                        params.gnums(1:n_phys));
obs_signed_distance = compute_fmm_map(grid_phys, occupancy_map);
obs_grid = Grid(params.gmin(1:n_phys), params.gmax(1:n_phys), params.gnums(1:n_phys));
obs_grid.SetData(obs_signed_distance);
params.reward_info.obstacles = obs_grid;
                     
% Setup theta info (convert to cell of cells).                      
params.reward_info.thetas = cell(1,numel(params.thetas));
for i=1:numel(params.thetas)
    params.reward_info.thetas{i} = params.thetas{i};
end

%% Create the Human Dynamical System.
% Initial state and dynamical system setup
params.initial_state = {1.74359,-1.94872,0.5};%{-0.923,-0.923,0.5};%{-2.97463,-0.923077,0.5};

% Params for Value Iteration. 
params.gamma = 0.95; 
params.eps = 0.01;

% MDP human.
gdisc3D = (params.gmax - params.gmin) ./ (params.gnums - 1);

% dt induced by discretization
params.vel = 0.6;
params.dt = gdisc3D(1)/params.vel;

% SGD update step.
params.alpha = 0.01;%0.01;
params.accuracy = 'high'; % valid: 'low', 'medium', 'high'
params.obs_padding = 3; % pads the size of the obstacle by number of cells 

params.dyn_sys = MDPHumanSGD3D(params.initial_state, ...
                                params.reward_info, ...
                                params.trueTheta, ...
                                params.uThresh, ...
                                gdisc3D, ...
                                params.gamma, ...
                                params.eps, ...
                                params.alpha, ...
                                params.w1, ...
                                params.g, ...
                                params.accuracy, ...
                                params.obs_padding);
                            
%% Target Set Setup
tol = 0.1;
centerTheta = params.trueTheta;

% xyoffset = -0.1;
% center = [0; 0; centerTheta];
% widths = [(params.gmax(1) - params.gmin(1)) + xyoffset; ...
%           (params.gmax(2) - params.gmin(2)) + xyoffset; 
%           tol];
% params.initial_value_fun = shapeRectangleByCenter(params.g, center, widths);

center = cell2mat(params.initial_state);
widths = [1,1,0.1];
params.initial_value_fun = shapeRectangleByCenter(params.g, center, widths);
% visSetIm(params.g, params.initial_value_fun);
% zlim([0,0.98])
% xlim([-4, 4])
% ylim([-4, 4])

%% Pack problem parameters
params.schemeData.grid = params.g;
params.schemeData.dynSys = params.dyn_sys;
params.schemeData.uMode = params.uMode;

%% Add obstacles to reachability computation too?
params.obstaclesInReachability = false;

%% Pack value function params
params.extraArgs.targets = params.initial_value_fun;
%params.extraArgs.stopInit = params.initial_state;

% 'none' or 'set' for backward reachable set (BRS)
% 'minVWithL' for backward reachable tube (BRT)
params.minWith = "minVWithL"; 

%% Optimal control reconstruction params
% Interpolate the value function when computing the opt ctrl?
params.extraArgsCtrl.interpolate = false;

end
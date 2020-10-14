function params = exp1_legibility()

%% Grid setup
params.gmin = [-6, -6, 0];
params.gmax = [6, 6, 1];
params.gnums = [30, 30, 25]; %[30, 30, 25];
params.g = createGrid(params.gmin, params.gmax, params.gnums);
params.extraArgs.g = params.g;
params.bdims = {3}; % dimension(s) which contain the belief

%% Joint Dynamics Setup.
params.thetas = {[1.5, -5.5], [-3, -3]}; %{[-5.5, -1], [1.8, -5.5]}; 
params.trueThetaIdx = 1;
%params.trueThetaIdx = 2;

%% Target Set Setup
% tol = 1-0.85;
% centerPgoal1 = (1-0.85)/2 + 0.85;
xyoffset = 0.1;
poffset = 0.1;
tol = 0.1;
if params.trueThetaIdx == 1
    center = [0; 0; 0.95];
    widths = [(params.gmax(1) - params.gmin(1)) + xyoffset;
              (params.gmax(2) - params.gmin(2)) + xyoffset;
              tol];
elseif params.trueThetaIdx == 2
    center = [0; 0; 0.05];
    widths = [(params.gmax(1) - params.gmin(1)) + xyoffset; ...
              (params.gmax(2) - params.gmin(2)) + xyoffset;
              tol];
end
params.initial_value_fun = shapeRectangleByCenter(params.g, center, widths);


%% Time vector
t0 = 1;
num_timesteps = 10;
params.tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
params.uMode = "min"; %"max"; % min or max
%params.uThresh = 0.2; %0.15; % threshold on P(u | x, g) -- e.g. 0.15;%0.14;%0.13;

params.uThresh = 0.0;
%params.uThresh = 0.05;
%params.uThresh = 0.15;
%params.uThresh = 0.2;

%% Plotting?
params.plot = true;        % Visualize the BRS and the optimal trajectory?

%% Save traj?
params.save = true;        % Save the BRS and the optimal trajectory?
params.exp_type = "exp_1";

%% Pack Reward Info

% 2D Grid for computing Q-function.
g_phys = createGrid(params.gmin(1:2)', ...
                    params.gmax(1:2)', ...
                    params.gnums(1:2)');
params.reward_info.g = g_phys; 

% Obstacles (based on interpolated occupancy grid) used in Q-function computation.
params.g2d = createGrid(params.gmin(1:2), params.gmax(1:2), params.gnums(1:2));
pts = [params.g2d.xs{1}(:), params.g2d.xs{2}(:)];
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/data/');
%map_name = 'cluttered_map_doorway.png';
map_name = 'map.png';
% obs_data = imread(strcat(data_path, map_name));
% n_phys = numel(params.gmin) - numel(params.bdims);
% params.reward_info.obstacles = get_obs_map(obs_data, ...
%                                         params.gmin(1:n_phys), ...
%                                         params.gmax(1:n_phys), ...
%                                         params.gnums(1:n_phys));
obs_data_2d = imread(strcat(data_path, map_name));
%obs_data_2d = rgb2gray(obs_data_3d);
params.obs_map_full = (obs_data_2d == 0) .* 1 + (obs_data_2d > 0) .* 0;
params.gimg = createGrid(params.gmin(1:2), params.gmax(1:2), size(params.obs_map_full));
obs_map = eval_u(params.gimg, params.obs_map_full, pts);
% make grid object to pass into reward info
grid_obs = Grid(params.gmin(1:2), params.gmax(1:2), params.gnums(1:2));
grid_obs.SetData(reshape(obs_map, params.gnums(1:2)));
params.reward_info.obstacles = grid_obs;                                    
                                    
                                    
% Setup theta info (convert to cell of cells).                      
params.reward_info.thetas = cell(1,numel(params.thetas));
for i=1:numel(params.thetas)
    params.reward_info.thetas{i} = {params.thetas{i}(1), params.thetas{i}(2)};
end

%% Create the Human Dynamical System.
% Initial state and dynamical system setup
params.initial_state = {0.2069, 0.2069, 0.5};
%{1.034, -0.6207, 0.5}; %{2.276,-2.69,0.5};% {4.345,-2.276,0.5}; %{1.5,1.5,0.5};

% Params for Value Iteration. 
params.gamma = 0.99; 
params.eps = 0.01;

% Variance on likelihood model: 
%   beta = 0 --> uniform dist, 
%   beta = 1 --> default 
%   beta = inf --> dirac delta on opt action)
params.beta = 1.0;

% MDP human.
gdisc3D = (params.gmax - params.gmin) ./ (params.gnums - 1);

% dt induced by discretization
params.vel = 0.6;
params.dt = gdisc3D(1)/params.vel;

% range of belief values
b_space = linspace(params.gmin(params.bdims{1}),params.gmax(params.bdims{1}),params.gnums(params.bdims{1}));
b_range = [b_space(2) b_space(numel(b_space)-1)];
        
params.dyn_sys = MDPHumanBelief3D(params.initial_state, ...
                                    params.reward_info, ...
                                    params.trueThetaIdx, ...
                                    params.uThresh, ...
                                    gdisc3D, ...
                                    params.gamma, ...
                                    params.eps, ...
                                    params.beta, ...
                                    b_range);
                                    
%% Pack problem parameters
params.schemeData.grid = params.g;
params.schemeData.dynSys = params.dyn_sys;
params.schemeData.uMode = params.uMode;

%% Add obstacles to reachability computation too?
% NOTE: 
% OBSTACLES IN REACHABILITY HAVEN'T BEEN 
% ADDED BASED ON THE NEW OBS LIST!
params.obstaclesInReachability = false;

% High confidence obstacle
% obs_center = [
%     0;
%     0;
%     (1-0.85)/2 + 0.85
% ];
% obs_width = [
%     (params.gmax(1) - params.gmin(1)) - xyoffset;
%     (params.gmax(2) - params.gmin(2)) - xyoffset;
%     0.1]; %(1-0.85)];
% 
% obs_center = [
%     0;
%     0;
%     (1-0.85)/2 + 0.85
% ];
% obs_width = [
%     (params.gmax(1) - params.gmin(1)) + xyoffset;
%     (params.gmax(2) - params.gmin(2)) + xyoffset;
%     0.1]; %(1-0.85)];
% obstacle_fun = -1 .* shapeRectangleByCenter(params.g, obs_center, obs_width);

if params.obstaclesInReachability
    params.extraArgs.obstacles = obstacle_fun;
    params.initial_value_fun = max(params.initial_value_fun, obstacle_fun);
end


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

function obs_fun = get_obstacle_fun(g, obstacles)
    obs_fun = zeros(size(g.xs{1}));
    for i=1:numel(obstacles)
        obs = obstacles{i};
        obs_center = [obs(1) + 0.5*obs(3); obs(2) + 0.5*obs(4); 0.5];
        obs_width = [obs(3); obs(4); 1.0];
        obs_fun = max(-1 .* shapeRectangleByCenter(g, obs_center, obs_width),obs_fun);
    end
end
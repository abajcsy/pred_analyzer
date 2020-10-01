function params = mdpHuman3DSimpleEnv()

%% Grid setup
params.gmin = [-4.5, -4.5, 0];
params.gmax = [4.5, 4.5, 1];
params.gnums = [20, 20, 20];
params.g = createGrid(params.gmin, params.gmax, params.gnums);
params.extraArgs.g = params.g;
params.bdims = {3}; % dimension(s) which contain the belief

%% Joint Dynamics Setup.
params.thetas = {[-2, 2], [2, 2]};
params.trueThetaIdx = 1;

%% Target Set Setup

% Rectangular prism centered at true b(g)
tol = 1-0.85;
centerPgoal1 = (1-0.85)/2 + 0.85;
xyoffset = -0.1;
center = [0; 0; centerPgoal1];
widths = [(params.gmax(1) - params.gmin(1)) + xyoffset; ...
          (params.gmax(2) - params.gmin(2)) + xyoffset; 
          tol];
params.initial_value_fun = shapeRectangleByCenter(params.g, center, widths);

% Cylinder centered at true goal
% xyoffset = 0.1;
% params.initial_value_fun = shapeCylinder(params.g,3,...
%     [params.thetas{params.trueThetaIdx}, 0.5], 0.5);

%% Time vector
t0 = 1;
num_timesteps = 10;
params.tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
params.uMode = "max"; % min or max
params.uThresh = 0.12; %0.13; % threshold on P(u | x, g) -- e.g. 0.16;0.15;%0.14;%0.13;

%% Plotting?
params.plot = true;        % Visualize the BRS and the optimal trajectory?

%% Pack Reward Info
% 2D Grid for computing Q-function.
g_phys = createGrid(params.gmin(1:2)', ...
                    params.gmax(1:2)', ...
                    params.gnums(1:2)');
params.reward_info.g = g_phys; 

% Obstacles used in Q-function computation.
% Axis-aligned rectangular obstacle convention is:
%       [lower_x, lower_y, width, height]
params.reward_info.obstacles = {[-1.5, 0.5, 1, 1],...
                                [-1.5, -2, 1, 2]};
                     
% Setup theta info (convert to cell of cells).                      
params.reward_info.thetas = cell(1,numel(params.thetas));
for i=1:numel(params.thetas)
    params.reward_info.thetas{i} = {params.thetas{i}(1), params.thetas{i}(2)};
end

%% Create the Human Dynamical System.
% Initial state and dynamical system setup
params.initial_state = {0,0,0.5};
%{-0.2105, -1.8947, 0.1}, 
%{0,0,0.1};
%{0.8,-2,0.5}; 
%{-2.2, 0, 0.1}; 
%{0.8,-2,0.1}; 
%{0,-3,0.1};

% Params for Value Iteration. 
params.gamma = 0.98; 
params.eps = 0.01;

% Variance on likelihood model: 
%   beta = 0 --> uniform dist, 
%   beta = 1 --> default 
%   beta = inf --> dirac delta on opt action)
params.beta = 1;

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

% % High confidence obstacle
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
% 
% if params.obstaclesInReachability
%     params.extraArgs.obstacles = obstacle_fun;
%     params.initial_value_fun = max(params.initial_value_fun, obstacle_fun);
% end

%% Pack value function params
params.extraArgs.targets = params.initial_value_fun;
%params.extraArgs.stopInit = params.initial_state;

% 'none' or 'set' for backward reachable set (BRS)
% 'minVWithL' for backward reachable tube (BRT)
params.minWith = 'none'; %"minVWithL"; 

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
function params = mdpHumanSGD3DSimpleEnv()

%% Grid setup
params.gmin = [-4, -4, 0];
params.gmax = [4, 4, 1];
params.gnums = [20, 20, 20];
params.g = createGrid(params.gmin, params.gmax, params.gnums);
params.bdims = 3; % dimension(s) which contain the belief

%% Time vector
t0 = 1;
num_timesteps = 15;
params.tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
params.uMode = "min";       % min or max
params.uThresh = 0.0;       % threshold on P(u | x, g) -- e.g. 0.15;%0.14;%0.13;

%% Plotting?
params.plot = true;         % Visualize the BRS and the optimal trajectory?

%% Joint Dynamics Setup.
params.thetas = {0,0.5,1}; % discrete set of thetas to precompute Q-func for
params.trueTheta = 0.5; 
params.w1 = 1;

%% Target Set Setup
tol = 0.2;
centerTheta = params.trueTheta;

xyoffset = 0.1;
center = [0; 0; centerTheta];
widths = [(params.gmax(1) - params.gmin(1)) + xyoffset; ...
          (params.gmax(2) - params.gmin(2)) + xyoffset; 
          tol];
params.initial_value_fun = shapeRectangleByCenter(params.g, center, widths);

%% Problem Setup
params.uMode = "min"; % min or max
params.uThresh = 0.0; % threshold on P(u | x, g)

%% Plotting?
params.plot = true;        % Visualize the BRS and the optimal trajectory?

%% Pack Reward Info

% 2D Grid for computing Q-function.
g_phys = createGrid(params.gmin(1:2)', ...
                    params.gmax(1:2)', ...
                    params.gnums(1:2)');
params.reward_info.g = g_phys; 
params.reward_info.goal = [-2; 2]; 
params.reward_info.goalRad = 1.3;

% Obstacles used in Q-function computation.
% Axis-aligned rectangular obstacle convention is:
%       [lower_x, lower_y, width, height]
params.reward_info.obstacles = {[-1.5, 0.5, 1, 1],...
                                [-1.5, -2, 1, 2]};
                     
% Setup theta info (convert to cell of cells).                      
params.reward_info.thetas = cell(1,numel(params.thetas));
for i=1:numel(params.thetas)
    params.reward_info.thetas{i} = params.thetas{i};
end

%% Create the Human Dynamical System.
% Initial state and dynamical system setup
params.initial_state = {-2, -1, 0.1};

% Params for Value Iteration. 
params.gamma = 0.98; 
params.eps = 0.01;

% MDP human.
gdisc3D = (params.gmax - params.gmin) ./ (params.gnums - 1);

% dt induced by discretization
params.vel = 0.6;
params.dt = gdisc3D(1)/params.vel;

% SGD update step.
params.alpha = 0.001;
params.accuracy = 'medium'; % valid: 'low', 'medium', 'high'
                                    
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
                                    params.accuracy);
                                
%% Pack problem parameters
params.schemeData.grid = params.g;
params.schemeData.dynSys = params.dyn_sys;
params.schemeData.uMode = params.uMode;

%% Add obstacles to reachability computation too?
params.obstaclesInReachability = false;

%% Pack value function params
params.extraArgs.targets = params.initial_value_fun;
params.extraArgs.stopInit = params.initial_state;

% 'none' or 'set' for backward reachable set (BRS)
% 'minVWithL' for backward reachable tube (BRT)
params.minWith = "minVWithL"; 

%% Optimal control reconstruction params
% Interpolate the value function when computing the opt ctrl?
params.extraArgsCtrl.interpolate = false;

end
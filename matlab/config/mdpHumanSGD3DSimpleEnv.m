function params = mdpHumanSGD3DSimpleEnv()

%% Grid setup
params.gmin = [-4, -4, 0];
params.gmax = [4, 4, 1];
params.gnums = [20, 20, 20];
params.g = createGrid(params.gmin, params.gmax, params.gnums);
params.bdims = 3; % dimension(s) which contain the belief

%% Time vector
t0 = 1;
num_timesteps = 10;
params.tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
params.uMode = "min";       % min or max
params.uThresh = 0.0;       % threshold on P(u | x, g) -- e.g. 0.15;%0.14;%0.13;

%% Plotting?
params.plot = true;         % Visualize the BRS and the optimal trajectory?

%% Joint Dynamics Setup.
params.thetas = {0,0.5,1};
params.trueTheta = 1;
params.w1 = 1;

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
params.reward_info.obstacles = {[-1.5, -2, 1, 2]};
%{[-1.5, 0.5, 1, 1],...
%[-1.5, -2, 1, 2]};
                     
% Setup theta info (convert to cell of cells).                      
params.reward_info.thetas = cell(1,numel(params.thetas));
for i=1:numel(params.thetas)
    params.reward_info.thetas{i} = params.thetas{i};
end

%% Create the Human Dynamical System.
% Initial state and dynamical system setup
params.initial_state = {-0.2105, -1.8947, 0.1};

% Params for Value Iteration. 
params.gamma = 0.98; 
params.eps = 0.01;

% MDP human.
gdisc3D = (params.gmax - params.gmin) ./ (params.gnums - 1);

% dt induced by discretization
params.vel = 0.6;
params.dt = gdisc3D(1)/params.vel;

% SGD update step.
params.alpha = 0.1;
                                    
params.dyn_sys = MDPHumanSGD3D(params.initial_state, ...
                                    params.reward_info, ...
                                    params.trueTheta, ...
                                    params.uThresh, ...
                                    gdisc3D, ...
                                    params.gamma, ...
                                    params.eps, ...
                                    params.alpha, ...
                                    params.w1);
                                
end
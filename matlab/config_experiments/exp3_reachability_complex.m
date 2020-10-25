function params = exp3_reachability_complex()

%% Grid setup
params.gmin = [-7.75, -7.75, 0, 0]; 
params.gmax = [7.75, 7.75, 2*pi, 1]; 
params.gnums = [35, 35, 17, 20]; %[40, 40, 17, 20]; 
pdDims = 3;
params.g = createGrid(params.gmin, params.gmax, params.gnums, pdDims);
params.bdims = {4}; % dimension(s) which contain the belief

%% Control Policy Parameterization Info.
params.thetas = {[-6.5, 1.83, pi], [-1.83, -6.5, 3*pi/2]};
params.trueThetaIdx = 2;

%% Target Set Setup
if params.trueThetaIdx == 1
    % Since third state is b(theta = 1), then when the true
    % human is optimizing for theta = 1, then target is 
    %       b(theta = 1) = 0.9
    centerPgoal1 = 0.95;
elseif params.trueThetaIdx == 2
    % Since third state is b(theta = 1), then when the true
    % human is optimizing for theta = 2, then target is 
    %       b(theta = 1) = 0.1 --> b(theta = 2) = 0.9
    centerPgoal1 = 0.05;
else 
    error('Invalid true theta index: %f!\n', params.trueThetaIdx);
end
xyoffset = -0.05;
phioffset = -0.01;
tol = 0.09;
center = [0; 0; pi; centerPgoal1];
widths = [(params.gmax(1) - params.gmin(1)) + xyoffset; ...
          (params.gmax(2) - params.gmin(2)) + xyoffset; ...
          (params.gmax(3) - params.gmin(3)) + phioffset; ...
          tol];
params.initial_value_fun = shapeRectangleByCenter(params.g, center, widths);

%% Time vector
t0 = 1;
num_timesteps = 35;
params.tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
params.uMode = "max"; % min or max
params.uThresh = 0.25; % threshold on P(u | x, g)

%% Plotting?
params.plot = true;        % Visualize the BRS and the optimal trajectory?

%% Pack Reward Info

% 2D Grid for computing Q-function.
g_phys = createGrid(params.gmin(1:3)', ...
                    params.gmax(1:3)', ...
                    params.gnums(1:3)');
params.reward_info.g = g_phys; 

% Obstacles used in Q-function computation.
% Axis-aligned rectangular obstacle convention is:
%       [lower_x, lower_y, width, height]
params.reward_info.obstacles = {[-7.75, -7.75, 4.1, 4.1]...
                                [3.65, -7.75, 4.1, 4.1], ...
                                [-7.75, 3.65, 4.1, 4.1], ...
                                [3.65, 3.65, 4.1, 4.1]};                         
                     
% Setup theta info (convert to cell of cells).                      
params.reward_info.thetas = cell(1,numel(params.thetas));
for i=1:numel(params.thetas)
    params.reward_info.thetas{i} = {params.thetas{i}(1), ...
                                    params.thetas{i}(2), ...
                                    params.thetas{i}(3)};
end

%% Create the Human Dynamical System.
% Initial state and dynamical system setup
params.initial_state = {6, 1.83, pi, 0.5};
% All relevant initial human states.
% all_h_x0s = {[6, 1.83, pi, 0.5], ...
%              [6, 1.83, pi, 0.1], ...
%              [6, 1.83, pi, 0.9]}     

% Params for Value Iteration. 
params.gamma = 0.9; % 0.98
params.eps = 0.01;

% Variance on likelihood model: 
%   beta = 0 --> uniform dist, 
%   beta = 1 --> default 
%   beta = inf --> dirac delta on opt action)
params.beta = 2; % need peakier dist to disambiguate controls.

% State space discretization
gdisc4D = (params.gmax - params.gmin) ./ (params.gnums - 1);

% dt induced by discretization
params.vel = 6; % Car's driving speed (m/s)
% NOTE THIS IS AN OVERAPPROX OF HOW AGGRESSIVELY THE REAL VEHICLE CAN TURN. 
% We need *at least* 3.5 rad/sec for the grid to be able to handle this...
params.max_ang_vel = 3.5; % %1.855; %0.8917; 
params.v_range = [4, params.vel]; %[params.vel/2, params.vel]; % Car's driving speed (m/s)
params.angular_range = [-params.max_ang_vel, 0, params.max_ang_vel];
params.dt = gdisc4D(1)/params.v_range(1); 

% range of belief values
b_space = linspace(params.gmin(params.bdims{1}),params.gmax(params.bdims{1}),params.gnums(params.bdims{1}));
params.b_range = [b_space(2) b_space(numel(b_space)-1)];

% MDP human.
params.dyn_sys = CarHumanBelief4DFull(params.initial_state, ...
                                    params.reward_info, ...
                                    params.trueThetaIdx, ...
                                    params.uThresh, ...
                                    gdisc4D, ...
                                    params.gnums, ...
                                    params.gamma, ...
                                    params.eps, ...
                                    params.beta, ...
                                    params.v_range, ...
                                    params.angular_range, ...
                                    params.dt, ...
                                    params.b_range);
         
%% Pack problem parameters
params.schemeData.grid = params.g;
params.schemeData.dynSys = params.dyn_sys;
params.schemeData.uMode = params.uMode;

%% Add obstacles to reachability computation too?
% NOTE: 
% OBSTACLES IN REACHABILITY HAVEN'T BEEN 
% ADDED BASED ON THE NEW OBS LIST!
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
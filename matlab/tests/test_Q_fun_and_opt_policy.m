clear all
clf
close all

%% Grid setup
gmin = [-4, -4, 0];
gmax = [4, 4, 1];
gnums = [20,20,20]; %[40, 40, 20];
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
num_timesteps = 10;
tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
uMode = "min"; % min or max
uThresh = 0.0;

%% Plotting?
plot = true;        % Visualize the BRS and the optimal trajectory?

%% Joint Dynamics Setup.
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;

%% Pack Reward Info

% 2D Grid for computing Q-function.
g_phys = createGrid([gmin(1), gmin(2)], [gmax(1), gmax(2)], [gnums(1), gnums(2)]);
reward_info.g = g_phys; 

% Obstacles for Q-function
% Rectangular obstacle convention is [lower_x, lower_y, width, height]
% reward_info.obstacles = {[-1.5, 0.5, 1, 1]...
%                          [-1.5, -2, 1, 2]};
reward_info.thetas = cell(1,numel(thetas));
for i=1:numel(thetas)
    reward_info.thetas{i} = {thetas{i}(1), thetas{i}(2)};
end

%% Create the Human Dynamical System.
% Initial state and dynamical system setup
initial_state = {0, 0, 0.1};

% Params for Value Iteration. 
gamma = 0.99; 
eps = 0.01;

% MDP human.
gdisc = (gmax - gmin) ./ (gnums - 1);
dyn_sys = MDPHumanBelief3D(initial_state, reward_info, trueThetaIdx, uThresh, gdisc, gamma, eps);

%% Plot optimal policy for each theta!
dyn_sys.plot_opt_policy(1);
dyn_sys.plot_opt_policy(2);
% Plot the probability of the actions at candidate state.
dyn_sys.plot_pu_given_x({-0.2, -1, 0.5}, thetas, trueThetaIdx);
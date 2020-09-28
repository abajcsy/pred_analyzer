%% File to compute belief initializations robust to model misspecification.
clear all
clf
close all

%% Setup

gmin = [-4.5, -4.5, 0];
gmax = [4.5, 4.5, 1];
gnums = [25, 25, 25];
g = createGrid(gmin, gmax, gnums);
    
thetas = {1,2};
uMode = "min";
thresh = 20; % out of 100
initial_state = {0,0,0.5};
minWith = "none";
model = "mdpHuman3DSimpleEnv";
out = "saved_value_funs/"+minWith+"_"+uMode+"_"+model+"_"+num2str(thresh)+".mat";

thresh = thresh/100;
%% Load or save value functions
if isfile(out)
    fprintf("Loading value function...\n");
    loaded_value_fun = load(out);
    value_funs_thetas = loaded_value_fun.value_funs_thetas;
else
    fprintf("Value function does not exist, creating and saving now...\n");
    value_funs_thetas = cell(1,numel(thetas)); % Holds value funs for each theta.

    for i=1:numel(thetas)
        trueThetaIdx = thetas{i};
        params = variable_mdpHuman3DSimpleEnv(uMode, thresh, trueThetaIdx, initial_state, minWith);

        [value_funs, tauOut, extraOuts] = ...
            DiscTimeHJIPDE_solve(params.initial_value_fun, ...
                                 params.tau, ...
                                 params.schemeData, ...
                                 params.minWith, ...
                                 params.extraArgs);

        value_funs_thetas{i} = value_funs;
    end
    
    save(out, 'value_funs_thetas');
end

%% Construct robust value fun

timesteps = numel(value_funs_thetas{1});

% robust_value_fun = \max_i value_fun_thetas{i}
robust_value_funs = cell(1, timesteps);

for i=1:timesteps
    v_fun_i = nan(size(value_funs_thetas{1}{i}));
    for theta=1:numel(thetas)
        v_fun_i = max(v_fun_i, value_funs_thetas{theta}{i});
    end
    v_grid_i = Grid(gmin,gmax,gnums);
    v_grid_i.SetData(v_fun_i);
    robust_value_funs{i} = v_grid_i;
end

%% Find earliest time when sets intersect at initial_state
gdisc = (gmax - gmin) ./ (gnums-1);
belief_disc = gmin(3):gdisc(3):gmax(3);
belief_initializations = [];

earliestTime = -1;
intersected = false;

for i=1:timesteps
    t = timesteps - i + 1;
    v_grid_t = robust_value_funs{t};
    for j=1:numel(belief_disc)
        b = belief_disc(j);
        if v_grid_t.GetDataAtReal({initial_state{1},initial_state{2},b}) <= 0
            intersected = true;
            belief_initializations = [belief_initializations b];
        end
    end
    
    if intersected
        earliestTime = t;
        break;
    end
end

%% Visualize set and print belief initializations
if earliestTime >= 0
    visSetIm(g,robust_value_funs{earliestTime}.data);
    fprintf("Robust initial beliefs: ");
    fprintf(num2str(belief_initializations));
    fprintf("\n");
else
    fprintf("No time at which scents intersect!")
end

%% Helper functions
function params = variable_mdpHuman3DSimpleEnv(uMode, thresh, trueThetaIdx, initial_state, minWith)

    %% Grid setup
    params.gmin = [-4.5, -4.5, 0];
    params.gmax = [4.5, 4.5, 1];
    params.gnums = [25, 25, 25];
    params.g = createGrid(params.gmin, params.gmax, params.gnums);
    params.extraArgs.g = params.g;
    params.bdims = 3; % dimension(s) which contain the belief

    %% Joint Dynamics Setup.
    params.thetas = {[-2, 2], [2, 2]};
    params.trueThetaIdx = trueThetaIdx;

    %% Target Set Setup
    tol = 0.1;
    if params.trueThetaIdx == 1
        centerPgoal1 = 0.95;
    else
        centerPgoal1 = 0.05;
    end
    
    xyoffset = 0.1;
    center = [0; 0; centerPgoal1];
    widths = [(params.gmax(1) - params.gmin(1)) + xyoffset; ...
              (params.gmax(2) - params.gmin(2)) + xyoffset; 
              tol];
    params.initial_value_fun = shapeRectangleByCenter(params.g, center, widths);

    % Cylinder centered at true goal
%     params.initial_value_fun = shapeCylinder(params.g,3,...
%         [params.thetas{params.trueThetaIdx}, 0.5], 0.5);

    %% Time vector
    t0 = 1;
    num_timesteps = 40;
    params.tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

    %% Problem Setup
    params.uMode = uMode; % min or max
    params.uThresh = thresh; % threshold on P(u | x, g) -- e.g. 0.15;%0.14;%0.13;

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
%     params.reward_info.obstacles = {[-1.5, 0.5, 1, 1],...
%                                     [-1.5, -2, 1, 2]};

    % Setup theta info (convert to cell of cells).                      
    params.reward_info.thetas = cell(1,numel(params.thetas));
    for i=1:numel(params.thetas)
        params.reward_info.thetas{i} = {params.thetas{i}(1), params.thetas{i}(2)};
    end

    %% Create the Human Dynamical System.
    % Initial state and dynamical system setup
    params.initial_state = initial_state;%{0,0,0.1};%{-0.2105, -1.8947, 0.1}; % {0.8,-2,0.1};

    % Params for Value Iteration. 
    params.gamma = 0.99; 
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
    b_space = linspace(params.gmin(params.bdims),params.gmax(params.bdims),params.gnums(params.bdims));
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
    obs_center = [
        0;
        0;
        (1-0.85)/2 + 0.85
    ];
    obs_width = [
        (params.gmax(1) - params.gmin(1)) + xyoffset;
        (params.gmax(2) - params.gmin(2)) + xyoffset;
        0.1]; %(1-0.85)];
    obstacle_fun = -1 .* shapeRectangleByCenter(params.g, obs_center, obs_width);

    if params.obstaclesInReachability
        params.extraArgs.obstacles = obstacle_fun;
        params.initial_value_fun = max(params.initial_value_fun, obstacle_fun);
    end


    %% Pack value function params
    params.extraArgs.targets = params.initial_value_fun;
%     params.extraArgs.stopInit = params.initial_state;

    % 'none' or 'set' for backward reachable set (BRS)
    % 'minVWithL' for backward reachable tube (BRT)
    params.minWith = minWith; 

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
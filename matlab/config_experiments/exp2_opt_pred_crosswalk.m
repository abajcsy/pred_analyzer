function params = exp2_opt_pred_crosswalk()
params.gmin = [-6,-6];
params.gmax = [6,6];
params.gnums = [45, 45];
params.pred_g = createGrid(params.gmin, params.gmax, params.gnums);
params.gdisc = (params.gmax-params.gmin) ./ (params.gnums-1);

% Compute timestep.
params.vel = 0.6;
params.dt = params.gdisc(1)/params.vel;

%% Prediction time info.
params.pred_hor = 8;                   % prediction horizon (in sec)
params.T = floor(params.pred_hor/params.dt);         % number of tsteps to predict for
params.discrete_times = 0:1:params.T;
params.real_times = params.discrete_times*params.dt;

%% Initial state and goal of human (in m)!
params.x0 = [1, -4];                    % initial position of human (in m)
params.goal = [1, 5];

% Obstacles (based on interpolated occupancy grid) used in Q-function computation.
params.binaryg2d = createGrid(params.gmin(1:2), params.gmax(1:2), params.gnums(1:2));
binary_map_pts = [params.binaryg2d.xs{1}(:), params.binaryg2d.xs{2}(:)];
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/data/');
% map_name = 'cluttered_map_doorway.png'; %'emptier_map.png'; 
obs_data_2d = ones(308,308);
params.obs_map_full = (obs_data_2d == 0) .* 1 + (obs_data_2d > 0) .* 0;
params.gimg = createGrid(params.gmin(1:2), params.gmax(1:2), size(params.obs_map_full));
% make grid object to pass into reward info
obs_binary_map = eval_u(params.gimg, params.obs_map_full, binary_map_pts);
grid_obs = Grid(params.gmin(1:2), params.gmax(1:2), params.gnums(1:2));
grid_obs.SetData(reshape(obs_binary_map, params.gnums(1:2)));

% Setup reward info & info for confidence-aware planning. 
params.reward_info.g = params.binaryg2d; 
params.reward_info.obstacles = grid_obs;
% Setup theta info (convert to cell of cells).                      
params.reward_info.theta = {params.goal(1), params.goal(2)};

params.gamma = 0.99; 
params.eps = 0.01;

% THIS IS SUCH A BIG HACK TO SAVE ME IMPLEMENTATION TIME.
params.gdisc = (params.gmax-params.gmin) ./ (params.gnums-1);
b_range = [0.01, 0.99];
z0 = {1.069, 0, 0.7762}; % doesn't really matter for our purposes.
joint_dyn_sys = MDPHumanConfidence3D(z0, ...
                                    params.reward_info, ...
                                    1, ...
                                    0.0, ...
                                    params.gdisc, ...
                                    params.gamma, params.eps, ...
                                    [0.1, 1], b_range);
params.dyn_sys = joint_dyn_sys;
params.q_fun = joint_dyn_sys.q_fun;
params.q_fun_grid = params.reward_info.g;

%% Create CONF-AWARE predictor.
fprintf('Setting up OPTIMAL PREDICTOR...\n');
params.predictor = OptPredictor(params.pred_g, params.goal, ...
                                params.q_fun, params.q_fun_grid);
fprintf('Done.\n');

end
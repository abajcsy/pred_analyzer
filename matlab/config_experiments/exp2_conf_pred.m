function params = exp2_conf_pred()
params.gmin = [-6,-6];
params.gmax = [6,6];
params.gnums = [45, 45]; %[30,30];
params.pred_g = createGrid(params.gmin, params.gmax, params.gnums);
params.gdisc = (params.gmax-params.gmin) ./ (params.gnums-1);

% Compute timestep.
params.vel = 0.6;
params.dt = params.gdisc(1)/params.vel;

%% Create all beta values.
params.betas = [0.1, 1];

%% Prediction time info.
params.pred_hor = 8;                   % prediction horizon (in sec)
params.T = floor(params.pred_hor/params.dt);         % number of tsteps to predict for
params.discrete_times = 0:1:params.T;
params.real_times = params.discrete_times*params.dt;

%% Initial state and goal of human (in m)!
params.x0 = [2.6, 1]; %[2, 0];                    % initial position of human (in m)
params.goal = [-5.5, -1];
params.beta_prior = [0.1,0.9]; % b(beta = low_conf), b(beta = high_conf)

% Obstacles (based on interpolated occupancy grid) used in Q-function computation.
params.binaryg2d = createGrid(params.gmin(1:2), params.gmax(1:2), params.gnums(1:2));
binary_map_pts = [params.binaryg2d.xs{1}(:), params.binaryg2d.xs{2}(:)];
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/data/');
map_name = 'cluttered_map_doorway.png'; %'emptier_map.png'; 
obs_data_2d = imread(strcat(data_path, map_name));
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

%% Create CONF-AWARE predictor.
fprintf('Setting up CONFIDENCE-AWARE PREDICTOR...\n');
params.predictor = ConfAwarePredictor(params.pred_g, params.goal, ....
                                        params.betas, params.reward_info, ...
                                        params.gamma, params.eps);
fprintf('Done.\n');

end
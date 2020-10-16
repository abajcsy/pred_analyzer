function params = exp2_planner_baselines()

%% Grid representation.
params.gmin = [-6,-6]; % should take into accound theta too?
params.gmax = [6,6];
params.gnums = [40,40];
params.g2d = createGrid(params.gmin, params.gmax, params.gnums);

% 3D grid including orientation
pdDim = 3;
ntheta = 30;
offset_pi = 0.01;
params.g3d = createGrid([params.gmin,-pi+offset_pi], [params.gmax,pi], [params.gnums,ntheta], pdDim);

%% Trajectory Info
params.num_waypts = 60; % Note: should match the number of steps predicted. 
params.horizon = 8;
params.dt = params.horizon/(params.num_waypts-1);
params.goal = [4.7, -3.2, -pi/2, 0.01]; 

% 13 meters / second ~= 30 mph
% 8 meters / second ~= 18 mph (is the average speed at intersection) 
dv = 0.25;
params.max_linear_vel = 1.; %0.65; % turtlebot max linear velocity
params.max_angular_vel = 1.; %1.5; % actual turtlebot angular vel is = pi
params.footprint_rad = 0.354; % note: this isn't used rn.... 

%% Car info.
params.car_len = 0.354*2; %4.5; % in m
params.car_width = 0.354*2; %1.8; % in m
params.car_rad = 0.354;

%% Signed dist functions.
pts = [params.g2d.xs{1}(:), params.g2d.xs{2}(:)];
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/data/');
map_name = 'cluttered_map_doorway.png'; %'emptier_map.png'; 
%obs_data_3d = imread(strcat(data_path, map_name));
%obs_data_2d = rgb2gray(obs_data_3d);
obs_data_2d = imread(strcat(data_path, map_name));
params.obs_map_full = (obs_data_2d == 0) .* -100.0 + (obs_data_2d > 0) .* 0.0;
params.gimg = createGrid(params.gmin, params.gmax, size(params.obs_map_full));
obs_map = eval_u(params.gimg, params.obs_map_full, pts);
obs_map = reshape(obs_map, params.gnums);

% Obstacle costmap with -100 for being in the obstacle and 0 otherwise.                                 
params.sd_obs = obs_map;

%% Create goal signed distance (positive in goal, negative outside).
params.goal_radius = 1;
params.sd_goal = -1 .* shapeCylinder(params.g2d, 3, params.goal(1:2), params.goal_radius); % 2D function (x,y)
% params.sd_goal = -1 .* shapeCylinder(params.g3d, [], params.goal(1:3), params.goal_radius); % 3D function (x,y,theta)

%% Setup probability over model confidence
params.belief = [0.5, 0.5]; % b(beta = 0.1) and b(beta = 1)
params.pthresh = 0.1;

%% Create spline planner!
fprintf('Setting up SINGLE SPLINE ROBOT PLANNER...\n');
params.planner = ConfSplinePlanner(params.num_waypts, ...
                params.horizon, ...
                params.goal, ...
                params.max_linear_vel, ...
                params.max_angular_vel, ...
                params.footprint_rad, ...
                params.sd_obs, ...
                params.sd_goal, ...
                params.g2d, ...
                params.g3d, ...
                params.gmin, params.gmax, params.gnums, ...
                params.pthresh);
fprintf('Done.\n');
            
end
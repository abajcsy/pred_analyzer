function params = contingency_crosswalk()

%% Grid representation.
params.gmin = [-6,-6]; % should take into accound theta too?
params.gmax = [6,6];
params.gnums = [35,35];
params.g2d = createGrid(params.gmin, params.gmax, params.gnums);

% 3D grid including orientation
pdDim = 3;
ntheta = 10;
offset_pi = 0.01;
params.g3d = createGrid([params.gmin,-pi+offset_pi], [params.gmax,pi], [params.gnums,ntheta], pdDim);

%% Trajectory Info
params.num_waypts = 60; % match the number of waypts predicted.
params.horizon = 8;
params.dt = params.horizon/(params.num_waypts-1);
params.goal = [-2, -4, -pi/2, 0.01];

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

%% Augment the size of obstacles by the robot base size.
pr2_base_width = 0.68; % m
footprint_rad = pr2_base_width/2.;
gdisc = (params.gmax - params.gmin) ./ (params.gnums - 1);
num_cells_augment = round(footprint_rad/gdisc(1)); % convert meters to cells for augmenting obstacles. 

%% Signed dist functions.
pts = [params.g2d.xs{1}(:), params.g2d.xs{2}(:)];
repo = what('pred_analyzer');
obs_data_2d = make_obs();
% black (0,0,0) == obstacles in image and 
% white >(0,0,0) == free-space in image 
binary_obs_map = (obs_data_2d == 0) .* 1 + (obs_data_2d > 0) .* 0;
params.bin_obs_map = binary_obs_map;
% zeros == free-space and ones == obstacles.

% Augment obstacle size by footprint of robot.
SE = strel('square', 5); % TODO: This always augments by 1 cell (because PR2 base maps to this). 
augmented_obs_map = imdilate(binary_obs_map, SE);

% Create costmap from augmented map. 
params.obs_map_full = (augmented_obs_map == 1) .* -100.0 + (obs_data_2d == 0) .* 0.0;
params.gimg = createGrid(params.gmin, params.gmax, size(params.obs_map_full));
obs_map = eval_u(params.gimg, params.obs_map_full, pts);
obs_map = reshape(obs_map, params.gnums);

% Obstacle costmap with -100 for being in the obstacle and 0 otherwise.                                 
params.sd_obs = obs_map;

%% Create goal signed distance (positive in goal, negative outside).
params.goal_radius = 1;
params.sd_goal = -1 .* shapeCylinder(params.g2d, 3, params.goal(1:2), params.goal_radius); % 2D function (x,y)

%% Setup probability over model confidence
params.belief = [0.5, 0.5]; % b(beta = 0.1) and b(beta = 1)
params.pthresh = 0.1;

%% Create spline planner!
params.planner = ConfContingencyPlanner(params.num_waypts, ...
                params.horizon, ...
                params.goal, ...
                params.max_linear_vel, ...
                params.max_angular_vel, ...
                params.footprint_rad, ...
                params.sd_obs, ...
                params.sd_goal, ...
                params.g2d, ...
                params.g3d, ...
                params.belief, ...
                params.gmin, params.gmax, params.gnums, ...
                params.pthresh, ...
                dv);
end

function obs = make_obs()
    obs = ones(308,308);
    obs(end-100:end,1:70) = 0;
    obs(end-100:end,end-70:end) = 0;
%     imwrite(obs, 'data/sidewalk.png');
end
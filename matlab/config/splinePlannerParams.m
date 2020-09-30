function params = splinePlannerParams()

%% Load "predicted" trajectories for the human for each goal.
load('opt_spline_g1.mat')
load('opt_spline_g2.mat')

params.opt_spline_g1 = opt_spline_g1;
params.opt_spline_g2 = opt_spline_g2;

%% Grid representation.
params.gmin = [-6.5,-6.5]; % should take into accound theta too?
params.gmax = [6.5,6.5];
params.gnums = [20,20];
params.g2d = createGrid(params.gmin, params.gmax, params.gnums);

% 3D grid including orientation
pdDim = 3;
params.g3d = createGrid([params.gmin,-pi], [params.gmax, pi], [params.gnums,20], pdDim);

%% Trajectory Info
params.num_waypts = 50;
params.horizon = 30;
params.dt = params.horizon/(params.num_waypts-1);
% params.goal = [5.6, -2.25, 0, 0.01]; % g1
params.goal = [2.25, 5.6, pi/2, 0.01]; % g2

% 13 meters / second ~= 30 mph
% 8 meters / second ~= 18 mph (is the average speed at intersection) 
params.max_linear_vel = 8.; 
params.max_angular_vel = 1.;
params.footprint_rad = 0.2794; % note: this isn't used rn.... 

%% Car info.
params.car_len = 3; %4.5; % in m
params.car_width = 1.2; %1.8; % in m
params.car_rad = 1.2;

%% Signed dist functions.
% Axis-aligned rectangular obstacle convention is:
%       [lower_x, lower_y, width, height]
params.obstacles = {[-6.5, -6.5, 2, 2]...
            [4.5, -6.5, 2, 2], ...
            [-6.5, 4.5, 2, 2], ...
            [4.5, 4.5, 2, 2]};
params.sd_obs = nan(params.g2d.shape);

%% Create obstacle signed distance (zero outside obstacle, negative inside).
params.obs_padding = ones(1,2)*0.25;
for i=1:length(params.obstacles)
    info = params.obstacles{i};
    lower = info(1:2) - params.obs_padding;
    upper = info(1:2) + info(3:4) + params.obs_padding;
    
    sd = shapeRectangleByCorners(params.g2d, lower, upper);
    
    % convert the signed distance into a "bump distance"
    % where phi(x,u) = 0 if outside obstacle
    % and   phi(x,u) = signedDist inside obstacle.
    obs_mask = (params.g2d.xs{1} >= lower(1)) & ...
        (params.g2d.xs{2} >= lower(2)) & ...
        (params.g2d.xs{1} <= upper(1)) & ...
        (params.g2d.xs{2} <= upper(2)); 

    sd = sd .* obs_mask;

    params.sd_obs = min(params.sd_obs, sd);
end

%% ADD IN PENALTY IF YOU GO INTO OTHER LANE!
lower = [4.5,-6.5] - params.obs_padding;
upper = [6.5,6.5] + params.obs_padding;
params.other_lane_sd = shapeRectangleByCorners(params.g2d, lower, upper);
params.sd_obs = min(params.sd_obs, params.other_lane_sd);

%% Create goal signed distance (positive in goal, negative outside).
params.goal_radius = 1;
params.angle_radius = pi/4;
params.sd_goal = -1 .* shapeCylinder(params.g2d, 3, params.goal(1:2), params.goal_radius); % 2D function (x,y)
% params.sd_goal = -1 .* shapeCylinder(params.g3d, [], params.goal(1:3), params.goal_radius); % 3D function (x,y,theta)

%% Create spline planner!
params.planner = SplinePlanner(params.num_waypts, ...
                params.horizon, ...
                params.goal, ...
                params.max_linear_vel, ...
                params.max_angular_vel, ...
                params.footprint_rad, ...
                params.sd_obs, ...
                params.sd_goal, ...
                params.opt_spline_g1, ...
                params.opt_spline_g2, ...
                params.g2d, ...
                params.g3d, ...
                params.gmin, params.gmax, params.gnums);
end
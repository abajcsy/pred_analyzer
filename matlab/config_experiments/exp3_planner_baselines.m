function params = exp3_planner_baselines()

%% Grid representation.
params.gmin = [-7.75, -7.75]; % should take into accound theta too?
params.gmax = [7.75, 7.75];
params.gnums = [15,15];
params.g2d = createGrid(params.gmin, params.gmax, params.gnums);

% 3D grid including orientation
pdDim = 3;
ntheta = 15;
offset_pi = 0.01;
params.g3d = createGrid([params.gmin,-pi+offset_pi], [params.gmax,pi], [params.gnums,ntheta], pdDim);

%% Trajectory Info
params.num_waypts = 50; % Note: should match the number of steps predicted. 
params.horizon = 8;
params.dt = params.horizon/(params.num_waypts-1);
params.goal = [1.83, 6.5, pi/2, 0.01]; 

% 13 meters / second ~= 30 mph
% 8 meters / second ~= 18 mph (is the average speed at intersection) 
dv = 1;
params.max_linear_vel = 6.; 
params.max_angular_vel = 1.; 
params.footprint_rad = 0.2794; % note: this isn't used rn.... 

%% Car info.
params.car_len = 4.2; % in m
params.car_width = 1.8; % in m
params.car_rad = 1.2; %1.9; %2.1; %<-- choose radius = length/2. 1.2;

%% Signed dist functions.
% Axis-aligned rectangular obstacle convention is:
%       [lower_x, lower_y, width, height]
params.obstacles = {[-7.75, -7.75, 4.1, 4.1]...
            [3.65, -7.75, 4.1, 4.1], ...
            [-7.75, 3.65, 4.1, 4.1], ...
            [3.65, 3.65, 4.1, 4.1]};
params.raw_sd_obs = nan(params.g2d.shape);

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

    params.raw_sd_obs = min(params.raw_sd_obs, sd);
end

%% ADD IN PENALTY IF YOU GO INTO OTHER LANE!
lower = [3.65,-7.75] - params.obs_padding;
upper = [7.75,7.75] + params.obs_padding;
params.other_lane_sd = shapeRectangleByCorners(params.g2d, lower, upper);
params.sd_obs = min(params.raw_sd_obs, params.other_lane_sd);

%% ADD ANOTHER PENALTY IF YOU GO INTO OPPOSING LANE
lower = [-3.65, 3.65] - params.obs_padding;
upper = [0, 7.75] + params.obs_padding;
params.final_lane_sd = shapeRectangleByCorners(params.g2d, lower, upper);
params.sd_obs = min(params.sd_obs, params.final_lane_sd);

%% Create goal signed distance (positive in goal, negative outside).
params.goal_radius = 1;
params.sd_goal = -1 .* shapeCylinder(params.g2d, 3, params.goal(1:2), params.goal_radius); % 2D function (x,y)
% params.sd_goal = -1 .* shapeCylinder(params.g3d, [], params.goal(1:3), params.goal_radius); % 3D function (x,y,theta)

%% Setup probability over model confidence
params.goal_prior = [0.5, 0.5]; % b(g = g1) and b(g = g2)

%% Create spline planner!
fprintf('Setting up SINGLE SPLINE DRIVING PLANNER...\n');
%% Create spline planner!
params.planner = DrivingSplinePlanner(params.num_waypts, ...
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
                params.car_rad);
fprintf('Done.\n');

%% Create the predictors for each goal!
goal1 = [-6.5, 1.83, pi, 0.01]; % g1 
goal2 = [-1.83, -6.5, -pi/2, 0.01];  % g2

% Predictor for human going to g1
params.g1 = goal1;
params.sd_goal_g1 = -1 .* shapeCylinder(params.g2d, 3, params.g1(1:2), params.goal_radius); % 2D function (x,y)
params.predictor_g1 = DrivingSplinePlanner(params.num_waypts, ...
                    params.horizon, ...
                    params.g1, ...
                    params.max_linear_vel, ...
                    params.max_angular_vel, ...
                    params.footprint_rad, ...
                    params.raw_sd_obs, ...
                    params.sd_goal_g1, ...
                    params.g2d, ...
                    params.g3d, ...
                    params.gmin, params.gmax, params.gnums, ...
                    params.car_rad);
                
% Predictor for human going to g2                
params.g2 = goal2;
params.sd_goal_g2 = -1 .* shapeCylinder(params.g2d, 3, params.g2(1:2), params.goal_radius); % 2D function (x,y)
params.predictor_g2 = DrivingSplinePlanner(params.num_waypts, ...
                    params.horizon, ...
                    params.g2, ...
                    params.max_linear_vel, ...
                    params.max_angular_vel, ...
                    params.footprint_rad, ...
                    params.raw_sd_obs, ...
                    params.sd_goal_g2, ...
                    params.g2d, ...
                    params.g3d, ...
                    params.gmin, params.gmax, params.gnums, ...
                    params.car_rad);
            
end
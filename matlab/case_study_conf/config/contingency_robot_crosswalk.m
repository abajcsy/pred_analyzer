function params = contingency_robot_crosswalk()

%% Grid representation.
params.gmin = [-6,-6]; % should take into accound theta too?
params.gmax = [6,6];
params.gnums = [25,25];
params.g2d = createGrid(params.gmin, params.gmax, params.gnums);

% 3D grid including orientation
pdDim = 3;
ntheta = 10;
offset_pi = 0.01;
params.g3d = createGrid([params.gmin,-pi+offset_pi], [params.gmax,pi], [params.gnums,ntheta], pdDim);

%% Trajectory Info
params.num_waypts = 60; % match the number of waypts predicted.
params.horizon = 4;
params.dt = params.horizon/(params.num_waypts-1);
params.goal = [-4, 2, pi, 0.01];

% 13 meters / second ~= 30 mph
% 8 meters / second ~= 18 mph (is the average speed at intersection) 
dv = 1;
params.max_linear_vel = 6.; 
params.max_angular_vel = 1.; 
params.footprint_rad = 0.2794; % note: this isn't used rn.... 
% dv = 0.25;
% params.max_linear_vel = 1.; %0.65; % turtlebot max linear velocity
% params.max_angular_vel = 1.; %1.5; % actual turtlebot angular vel is = pi
% params.footprint_rad = 0.354; % note: this isn't used rn.... 

%% Car info.
% params.car_len = 0.354*2; %4.5; % in m
% params.car_width = 0.354*2; %1.8; % in m
% params.car_rad = 0.354;
params.car_len = 4.2; % in m
params.car_width = 1.8; % in m
params.car_rad = 1.2; %1.9; %2.1; %<-- choose radius = length/2. % 1.2;

%% Signed dist functions.
% Axis-aligned rectangular obstacle convention is:
%       [lower_x, lower_y, width, height]
params.obstacles = {[2, -7.75, 5.75, 4.1], ...
                    [2, 3.65, 5.75, 4.1]};
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
params.sd_obs = params.raw_sd_obs;

% %% ADD IN PENALTY IF YOU GO INTO OTHER LANE!
% lower = [3.65,-7.75] - params.obs_padding;
% upper = [7.75,7.75] + params.obs_padding;
% params.other_lane_sd = shapeRectangleByCorners(params.g2d, lower, upper);
% params.sd_obs = min(params.raw_sd_obs, params.other_lane_sd);
% 
% %% ADD ANOTHER PENALTY IF YOU GO INTO OPPOSING LANE
% lower = [-3.65, 3.65] - params.obs_padding;
% upper = [0, 7.75] + params.obs_padding;
% params.final_lane_sd = shapeRectangleByCorners(params.g2d, lower, upper);
% params.sd_obs = min(params.sd_obs, params.final_lane_sd);

% %% Signed dist functions.
% pts = [params.g2d.xs{1}(:), params.g2d.xs{2}(:)];
% repo = what('pred_analyzer');
% obs_data_2d = make_obs();
% % black (0,0,0) == obstacles in image and 
% % white >(0,0,0) == free-space in image 
% binary_obs_map = (obs_data_2d == 0) .* 1 + (obs_data_2d > 0) .* 0;
% params.bin_obs_map = binary_obs_map;
% % zeros == free-space and ones == obstacles.
% 
% % Augment obstacle size by footprint of robot.
% SE = strel('square', 5); % TODO: This always augments by 1 cell (because PR2 base maps to this). 
% augmented_obs_map = imdilate(binary_obs_map, SE);
% 
% % Create costmap from augmented map. 
% params.obs_map_full = (augmented_obs_map == 1) .* -100.0 + (obs_data_2d == 0) .* 0.0;
% params.gimg = createGrid(params.gmin, params.gmax, size(params.obs_map_full));
% obs_map = eval_u(params.gimg, params.obs_map_full, pts);
% obs_map = reshape(obs_map, params.gnums);
% 
% % Obstacle costmap with -100 for being in the obstacle and 0 otherwise.                                 
% params.sd_obs = obs_map;

%% Create goal signed distance (positive in goal, negative outside).
params.goal_radius = 1;
params.sd_goal = -1 .* shapeCylinder(params.g2d, 3, params.goal(1:2), params.goal_radius); % 2D function (x,y)

%% Setup probability over model confidence
params.belief = [0.1, 0.9]; % b(beta = 0.1) and b(beta = 1)
params.pthresh = 0.05;

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
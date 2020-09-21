clear all

%% Grid representation.
gmin = [-6.5,-6.5]; % should take into accound theta too?
gmax = [6.5,6.5];
gnums = [50,50];
g2d = createGrid(gmin, gmax, gnums);

%% Trajectory Info
num_waypts = 8;
horizon = 8;
% goal = [5.6, -2.25, 0, 0.01]; %[-3.6, 1, pi]; % g1
goal = [2.25, 5.6, pi/2, 0.01]; %[1, 3.6, pi/2]; % g2

% 13 meters / second ~= 30 mph
% 8 meters / second ~= 18 mph (is the average speed at intersection) 
max_linear_vel = 8.; 
max_angular_vel = 1.;
footprint_rad = 0.2794; % note: this isn't used rn.... 

%% Car info.
car_len = 3; %4.5; % in m
car_width = 1.2; %1.8; % in m

%% Signed dist functions.
% Axis-aligned rectangular obstacle convention is:
%       [lower_x, lower_y, width, height]
obstacles = {[-6.5, -6.5, 2, 2]...
            [4.5, -6.5, 2, 2], ...
            [-6.5, 4.5, 2, 2], ...
            [4.5, 4.5, 2, 2]};
sd_obs = nan(g2d.shape);

% Create obstacle signed distance (positive outside obstacle, negative inside).
% TODO: may need to shape this to have it be a bump distance function?
obs_padding = ones(1,2)*0.25;
for i=1:length(obstacles)
    info = obstacles{i};
    lower = info(1:2) - obs_padding;
    upper = info(1:2) + info(3:4) + obs_padding;
    
    sd = shapeRectangleByCorners(g2d, lower, upper);
    
    % convert the signed distance into a "bump distance"
    % where phi(x,u) = 0 if outside obstacle
    % and   phi(x,u) = signedDist inside obstacle.
    obs_mask = (g2d.xs{1} >= lower(1)) & ...
        (g2d.xs{2} >= lower(2)) & ...
        (g2d.xs{1} <= upper(1)) & ...
        (g2d.xs{2} <= upper(2)); 

    sd = sd .* obs_mask;

    sd_obs = min(sd_obs, sd);
end

% Create goal signed distance (positive in goal, negative outside).
goal_radius = 1;
sd_goal = -1 .* shapeCylinder(g2d, 3, goal(1:2), goal_radius);

%% Create spline planner!
planner = SplinePlanner(num_waypts, horizon, ...
                goal, ...
                max_linear_vel, ...
                max_angular_vel, ...
                footprint_rad, ...
                sd_obs, ...
                sd_goal, ...
                g2d, ...
                gmin, gmax, gnums);
            
%% Plan!
% note: need a non-zero starting velocity to avoid singularities in spline
start = [-5.6, -2.25, 0, 0.01]; 
opt_spline = planner.plan(start, goal);
            
%% Plot!
xs = opt_spline{1};
ys = opt_spline{2};
ths = atan2(ys(2:end) - ys(1:end-1), xs(2:end) - xs(1:end-1));
ths = [start(3), ths];
u1s = opt_spline{3};
u2s = opt_spline{4};

%% Plot velocity profiles
figure(1)
clf
hold on
plot(1:1:num_waypts, u1s, '-k', 'linewidth', 2);
plot([1,num_waypts], [max_linear_vel, max_linear_vel], '--b');
plot([1,num_waypts], [-max_linear_vel, -max_linear_vel], '--b');
title('linear velocity over time');
ylim([-max_linear_vel, max_linear_vel])

figure(2)
clf
hold on
plot(1:1:num_waypts, u2s, '-k', 'linewidth', 2);
plot([1,num_waypts], [max_angular_vel, max_angular_vel], '--b');
plot([1,num_waypts], [-max_angular_vel, -max_angular_vel], '--b');
title('angular velocity over time');
ylim([-max_angular_vel, max_angular_vel])

%% Plot the trajectory and environment. 
figure(3)
clf
hold on

% Plot the lane boundaries.
plot([gmin(1), -4.5], [0,0], '--y', 'linewidth', 4);
plot([4.5, gmax(1)], [0,0], '--y', 'linewidth', 4);
plot([0,0], [gmin(2), -4.5], '--y', 'linewidth', 4);
plot([0,0], [4.5, gmax(2)], '--y', 'linewidth', 4);

% Plot the trajectory. 
colors = linspace(0.1, 0.9, length(xs));
for i=1:length(xs)
    plot_car(xs(i), ys(i), ths(i), car_len, car_width, [colors(i),0,0]);
end
s = scatter(xs, ys, 'r', 'filled');
s.CData = [colors', zeros(length(xs), 1), zeros(length(xs), 1)];
% quiver(xs, ys, cos(ths), sin(ths), 'r');

% Plot obstacles.
for oi = 1:length(obstacles)
    obs_info = obstacles{oi};
    obs_min = obs_info(1:2);

    x_min = obs_min(1);
    y_min = obs_min(2);
    p_min = 0;
    l = [obs_info(3), ...
        obs_info(4), ...
        0];
    plotcube(l + [obs_padding*2, 0], [x_min y_min p_min] - [obs_padding, 0], 0.1, [0.8,0.8,0.8]);
    plotcube(l,[x_min y_min p_min], .5, [0.3 0.3 0.3]);
end

% Plot goal + goal region
scatter(goal(1), goal(2), 'r', 'filled');
quiver(goal(1), goal(2), cos(goal(3)), sin(goal(3)), 'r');
pos = [goal(1)-goal_radius, goal(2)-goal_radius, ...
       goal_radius*2, goal_radius*2];
rectangle('position', pos, ...
            'curvature', [1,1], ...
            'EdgeColor','r');
xlim([gmin(1),gmax(1)]);
ylim([gmin(2),gmax(2)]);
set(gcf, 'color', 'white')
view(0,90)
% grid on

%% Plots a car as a rotated rectangle. 
function plot_car(xc, yc, th, car_len, car_width, color)

rot_mat = [cos(th) -sin(th); ... 
           sin(th) cos(th)];

xoff = 0.5*car_len;
yoff = 0.5*car_width;

ur = [xc; yc] + rot_mat * [xoff; yoff];
ul = [xc; yc] + rot_mat * [- xoff; yoff]; 
dl = [xc; yc] + rot_mat * [- xoff; - yoff]; 
dr = [xc; yc] + rot_mat * [xoff; - yoff]; 

xs = [ur(1) ul(1) dl(1) dr(1)];
ys = [ur(2) ul(2) dl(2) dr(2)];

p = patch(xs,ys, color);
p.FaceColor = color;
p.FaceAlpha = 0.1;
p.EdgeColor = color;

end

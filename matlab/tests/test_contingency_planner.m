clear all
close all

%% Load up all the info.
params = contingencyPlannerParams();
            
%% Plan!
% note: need a non-zero starting velocity to avoid singularities in spline
branch_t = params.planner.dt*2; %1.5;
start = [-5.6, -2.25, 0, 0.01]; 
opt_plan = ...
    params.planner.contingency_plan(start, params.goal,  branch_t);

shared_spline = opt_plan{1};
spline_g1 = opt_plan{2};
spline_g2 = opt_plan{3};

%% Plot!
g1_ths = spline_g1{5};
g2_ths = spline_g2{5};

shared_xs = shared_spline{1};
shared_ys = shared_spline{2};
shared_ths = shared_spline{5};
shared_u1s = shared_spline{3};
shared_u2s = shared_spline{4};

% %% Plot velocity profiles
% figure(1)
% clf
% hold on
% plot(0:dt:horizon, u1s, '-k', 'linewidth', 2);
% plot([0,horizon], [max_linear_vel, max_linear_vel], '--b');
% plot([0,horizon], [-max_linear_vel, -max_linear_vel], '--b');
% title('linear velocity over time');
% xlabel('t (sec)');
% ylim([-max_linear_vel, max_linear_vel])
% 
% figure(2)
% clf
% hold on
% plot(0:dt:horizon, u2s, '-k', 'linewidth', 2);
% plot([0,horizon], [max_angular_vel, max_angular_vel], '--b');
% plot([0,horizon], [-max_angular_vel, -max_angular_vel], '--b');
% title('angular velocity over time');
% xlabel('t (sec)');
% ylim([-max_angular_vel, max_angular_vel])

%% Plot the trajectory and environment. 
figure(3)
clf
hold on

% Plot the lane boundaries.
plot([params.gmin(1), -4.5], [0,0], '--y', 'linewidth', 4);
plot([4.5, params.gmax(1)], [0,0], '--y', 'linewidth', 4);
plot([0,0], [params.gmin(2), -4.5], '--y', 'linewidth', 4);
plot([0,0], [4.5, params.gmax(2)], '--y', 'linewidth', 4);

% Plot the EGO CAR's trajectory. 
shared_r_colors = [linspace(0.1, 0.9, length(shared_xs))', ...
                   zeros([length(shared_xs),1]), ...
                   zeros([length(shared_xs),1])];
% shared traj
plot_traj(shared_xs, shared_ys, shared_ths, ...
            params.car_len, params.car_width, params.car_rad, ...
            shared_r_colors, 0, branch_t);

% traj for g1
g1_r_colors = [linspace(0.1, 0.9, length(spline_g1{1}))', ...
                linspace(0.1, 0.9, length(spline_g1{1}))', ...
                zeros([length(spline_g1{1}),1])];
plot_traj(spline_g1{1}, spline_g1{2}, g1_ths, ...
            params.car_len, params.car_width, params.car_rad, g1_r_colors, ...
            branch_t + params.dt, params.horizon);

% traj for g2
g2_r_colors = [zeros([length(spline_g2{1}),1]), ...
                linspace(0.1, 0.9, length(spline_g2{1}))', ...
                linspace(0.1, 0.9, length(spline_g2{1}))'];
plot_traj(spline_g2{1}, spline_g2{2}, g2_ths, ...
            params.car_len, params.car_width, params.car_rad, g2_r_colors, ...
            branch_t + params.dt, params.horizon);

%% Plot the PREDICTED trajectories.
g1_human_colors = [linspace(0.1, 0.9, length(params.opt_spline_g1{1}))', ...
                linspace(0.1, 0.9, length(params.opt_spline_g1{1}))', ...
                zeros([length(params.opt_spline_g1{1}),1])];
g2_human_colors = [zeros([length(params.opt_spline_g2{1}),1]), ...
                linspace(0.1, 0.9, length(params.opt_spline_g2{1}))', ...
                linspace(0.1, 0.9, length(params.opt_spline_g2{1}))'];   
            
pred_g1_ths = atan2(params.opt_spline_g1{2}(2:end) - params.opt_spline_g1{2}(1:end-1), ...
                    params.opt_spline_g1{1}(2:end) - params.opt_spline_g1{1}(1:end-1));
pred_g1_ths = [pred_g1_ths, pi];

pred_g2_ths = atan2(params.opt_spline_g2{2}(2:end) - params.opt_spline_g2{2}(1:end-1), ...
                    params.opt_spline_g2{1}(2:end) - params.opt_spline_g2{1}(1:end-1));
pred_g2_ths = [pred_g2_ths, -pi/2];

plot_traj(params.opt_spline_g1{1}, params.opt_spline_g1{2}, pred_g1_ths, ...
            params.car_len, params.car_width, params.car_rad, g1_human_colors, ...
            0, params.horizon);
plot_traj(params.opt_spline_g2{1}, params.opt_spline_g2{2}, pred_g2_ths, ..., 
            params.car_len, params.car_width, params.car_rad, g2_human_colors, ...
            0, params.horizon);

%% Plot obstacles.
for oi = 1:length(params.obstacles)
    obs_info = params.obstacles{oi};
    obs_min = obs_info(1:2);

    x_min = obs_min(1);
    y_min = obs_min(2);
    p_min = 0;
    l = [obs_info(3), ...
        obs_info(4), ...
        0];
    plotcube(l + [params.obs_padding*2, 0], [x_min y_min p_min] - [params.obs_padding, 0], 0.1, [0.8,0.8,0.8]);
    plotcube(l,[x_min y_min p_min], .5, [0.3 0.3 0.3]);
end
contour(params.g2d.xs{1}, params.g2d.xs{2}, params.other_lane_sd, [0,0], ...
    'Color', [0.6,0.6,0.6], 'LineStyle', ':');

% Plot goal + goal region
scatter(params.goal(1), params.goal(2), 'r', 'filled');
quiver(params.goal(1), params.goal(2), cos(params.goal(3)), sin(params.goal(3)), 'r');
pos = [params.goal(1)-params.goal_radius, params.goal(2)-params.goal_radius, ...
       params.goal_radius*2, params.goal_radius*2];
rectangle('position', pos, ...
            'curvature', [1,1], ...
            'EdgeColor','r');
xlim([params.gmin(1),params.gmax(1)]);
ylim([params.gmin(2),params.gmax(2)]);
set(gcf, 'color', 'white')
view(0,90)
set(gcf, 'position', [0,0,800,800])
% grid on

%% Plots the trajectory
function plot_traj(xs, ys, ths, car_len, car_width, car_rad, colors, start_t, end_t)
%     for i=1:length(xs)
%         %plot_car(xs(i), ys(i), ths(i), car_len, car_width, colors(i));
%         pos = [xs(i)-car_rad, ys(i)-car_rad, ...
%                car_rad*2, car_rad*2];
%         rectangle('position', pos, ...
%                     'curvature', [1,1], ...
%                     'EdgeColor', colors(i, :));
%     end
    s = scatter(xs, ys, 'r', 'filled');
    s.CData = colors; %[colors', zeros(length(xs), 1), zeros(length(xs), 1)];
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', colors(end,:));
    q.ShowArrowHead = 'off';
    q.AutoScale = 'off';
    q.AutoScaleFactor = 0.5;
    text(xs(1)+0.1, ys(1), strcat('t=',num2str(start_t)));
    text(xs(end)+0.1, ys(end), strcat('t=',num2str(end_t)));
end

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

close all
clear all

%% Load all spline plans.
% load('splines_bg101.mat');
load('safe_spline.mat');

%% Type of spline?
splineType = 'safe'; % 'safe' or 'contingency'

%% Load up all the info.
params = contingencyPlannerParams();

if strcmp(splineType, 'contingency')
    shared_spline = opt_plan{1};
    spline_g1 = opt_plan{2};
    spline_g2 = opt_plan{3};

    g1_ths = spline_g1{5};
    g2_ths = spline_g2{5};

    shared_xs = shared_spline{1};
    shared_ys = shared_spline{2};
    shared_ths = shared_spline{5};
    shared_u1s = shared_spline{3};
    shared_u2s = shared_spline{4};
end

%% Colors.
shared_r_colors = [linspace(0.1, 0.9, params.num_waypts)', ...
                   zeros([params.num_waypts,1]), ...
                   zeros([params.num_waypts,1])];
               
g1_human_colors = [linspace(0.1, 0.9, length(params.opt_spline_g1{1}))', ...
                    linspace(0.1, 0.9, length(params.opt_spline_g1{1}))', ...
                    zeros([length(params.opt_spline_g1{1}),1])];

% extract thetas.
pred_g1_ths = atan2(params.opt_spline_g1{2}(2:end) - params.opt_spline_g1{2}(1:end-1), ...
    params.opt_spline_g1{1}(2:end) - params.opt_spline_g1{1}(1:end-1));
pred_g1_ths = [pred_g1_ths, pi];

%% Plot goal + goal region
figure
hold on
% Plot the lane boundaries.
plot([params.gmin(1), -4.5], [0,0], '--y', 'linewidth', 4);
plot([4.5, params.gmax(1)], [0,0], '--y', 'linewidth', 4);
plot([0,0], [params.gmin(2), -4.5], '--y', 'linewidth', 4);
plot([0,0], [4.5, params.gmax(2)], '--y', 'linewidth', 4);

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

%% Simulate the trajectories!
for i=1:params.num_waypts
    % Plot EGO car
    if strcmp(splineType, 'safe')
        qr = quiver(opt_spline{1}(i), opt_spline{2}(i), ...
                    cos(opt_spline{5}(i)), sin(opt_spline{5}(i)), ...
                    'Color', shared_r_colors(i,:));
    else
        if i <= length(shared_spline{1})
            qr = quiver(shared_spline{1}(i), shared_spline{2}(i), ...
                        cos(shared_spline{5}(i)), sin(shared_spline{5}(i)), ...
                        'Color', shared_r_colors(i,:));
        else
            qr = quiver(spline_g1{1}(i), spline_g1{2}(i), ...
                        cos(spline_g1{5}(i)), sin(spline_g1{5}(i)), ...
                        'Color', shared_r_colors(i,:));
        end
    end
    
    % Plot body of the EGO vehicle. 
    pos = [opt_spline{1}(i)-params.car_rad, opt_spline{2}(i)-params.car_rad, ...
           params.car_rad*2, params.car_rad*2];
    rr = rectangle('position', pos, ...
                'curvature', [1,1], ...
                'EdgeColor', shared_r_colors(i, :));
    
    qr.ShowArrowHead = 'on';
    qr.AutoScale = 'off';
    qr.AutoScaleFactor = 0.5;
    qr.Marker = 'o';
    qr.MarkerFaceColor = shared_r_colors(i,:);
    
    % Plot car going to g1.
    qh = quiver(params.opt_spline_g1{1}(i), params.opt_spline_g1{2}(i), ...
                cos(pred_g1_ths(i)), sin(pred_g1_ths(i)), ...
                'Color', g1_human_colors(i,:));
    qh.ShowArrowHead = 'on';
    qh.AutoScale = 'off';
    qh.AutoScaleFactor = 0.5;  
    qh.Marker = 'o';
    qh.MarkerFaceColor = g1_human_colors(i,:);
    
    pos = [params.opt_spline_g1{1}(i)-params.car_rad, params.opt_spline_g1{2}(i)-params.car_rad, ...
           params.car_rad*2, params.car_rad*2];
    rh = rectangle('position', pos, ...
                'curvature', [1,1], ...
                'EdgeColor', g1_human_colors(i, :));
    
    pause(0.1);
    delete(qr);
    delete(rr);
    delete(qh);
    delete(rh);
end
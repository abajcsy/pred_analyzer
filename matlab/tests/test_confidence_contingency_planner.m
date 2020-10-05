clear all
close all

%% Load up all the info.
params = exp2_contingency_planner();
r_start = [1, 4, -pi/4, 0.01]; 

% PLOT!
figure(1)
hold on

% Plot environment.
bandw_cmap = [0,0,0;1,1,1]; %[1,1,1;0,0,0];
colormap(bandw_cmap)
%ph = pcolor(params.g2d.xs{1}, params.g2d.xs{2}, params.sd_obs);
ph = pcolor(params.gimg.xs{1}, params.gimg.xs{2}, params.obs_map_full);
set(ph, 'EdgeColor', 'none');

% plot robot goal.
qrg = quiver(params.goal(1), params.goal(2), cos(params.goal(3)), sin(params.goal(3)), 'k');
qrg.Marker = 'o';
qrg.MarkerFaceColor = 'k';
% plot robot state.
% qrs = quiver(r_start(1), r_start(2), cos(r_start(3)), sin(r_start(3)), 'k');
% qrs.Marker = 'o';
% qrs.MarkerEdgeColor = 'k';
% qrs.MarkerFaceColor = 'w';
% plot human goal.
h_goal = [-3.5, 0];
h_x0 = [2, 0];  
% plot human goal.
scatter(h_goal(1), h_goal(2), 'r');
% plot human state
scatter(h_x0(1), h_x0(2), 'r', 'filled');
xlim([params.gmin(1), params.gmax(1)])
ylim([params.gmin(2), params.gmax(2)])
            
%% Plan!
% note: need a non-zero starting velocity to avoid singularities in spline
branch_t = 1.38; % for 50/50 prior 
opt_plan = ...
    params.planner.contingency_plan(r_start, params.goal, branch_t);

% load('conf_cont_plan.mat');

shared_spline = opt_plan{1};
spline_opt = opt_plan{2};
spline_frs = opt_plan{3};

%% Plot!
q = quiver(shared_spline{1}, shared_spline{2}, ...
    cos(shared_spline{5}), sin(shared_spline{5}), 'k');
q.Marker = 'o';
q.MarkerFaceColor = 'k';
q.MarkerEdgeColor = 'k';
q.ShowArrowHead = 'off';
q.AutoScale = 'off';
q.AutoScaleFactor = 0.5;

q = quiver(spline_opt{1}, spline_opt{2}, ...
    cos(spline_opt{5}), sin(spline_opt{5}), 'r');
q.Marker = 'o';
q.MarkerFaceColor = 'r';
q.MarkerEdgeColor = 'r';
q.ShowArrowHead = 'off';
q.AutoScale = 'off';
q.AutoScaleFactor = 0.5;

q = quiver(spline_frs{1}, spline_frs{2}, ...
    cos(spline_frs{5}), sin(spline_frs{5}), 'b');
q.Marker = 'o';
q.MarkerFaceColor = 'b';
q.MarkerEdgeColor = 'b';
q.ShowArrowHead = 'off';
q.AutoScale = 'off';
q.AutoScaleFactor = 0.5;

xlim([params.gmin(1), params.gmax(1)])
ylim([params.gmin(2), params.gmax(2)])
set(gcf, 'color', 'w');
set(gcf, 'position', [0,0,800,800])

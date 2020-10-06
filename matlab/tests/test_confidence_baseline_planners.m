clear all
close all

%% Load up all the info.
params = exp2_planner_baselines();
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
% plot human goal.
h_goal = [-3.5, 0];
h_x0 = [2, 0];  
% plot human goal.
scatter(h_goal(1), h_goal(2), 'r');
% plot human state
scatter(h_x0(1), h_x0(2), 'r', 'filled');
xlim([params.gmin(1), params.gmax(1)])
ylim([params.gmin(2), params.gmax(2)])

%% -------------- debugging ---------------- %%
%figure(2)
%params.planner.plot_human_preds('conf')
% --------------- debugging ---------------- %
            
%% Plan!
% note: need a non-zero starting velocity to avoid singularities in spline

coll_check = 'conf'; 
opt_plan = ...
    params.planner.plan(r_start, params.goal, coll_check);

%% Plot!
if strcmp(coll_check, 'opt')
    color = 'r';
elseif strcmp(coll_check, 'frs') || strcmp(coll_check, 'all')
    color = 'b';
else % if conf-aware
    color = 'm';
end
q = quiver(opt_plan{1}, opt_plan{2}, ...
    cos(opt_plan{5}), sin(opt_plan{5}), color);
q.Marker = 'o';
q.MarkerFaceColor = color;
q.MarkerEdgeColor = color;
q.ShowArrowHead = 'off';
q.AutoScale = 'off';
q.AutoScaleFactor = 0.5;

xlim([params.gmin(1), params.gmax(1)])
ylim([params.gmin(2), params.gmax(2)])
set(gcf, 'color', 'w');
set(gcf, 'position', [0,0,800,800])
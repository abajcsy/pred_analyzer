clear all
close all

load('exp_3_heuristic_baseline_trajs_brancht0.3265.mat');

% ri = 1, hi = 1, pgi = 1, gi = 1

gi = 1;
i = 1;

goal = all_h_goals{gi};
g1_preds = all_g1_preds{i};
g2_preds = all_g2_preds{i};
r_plan = all_plans{i};

if strcmp(goal, 'g1')
    human_traj = g1_preds;
else
    human_traj = g2_preds;
end

%% PLOT!
figure(1)
hold on
% plot robot goal.
qrg = quiver(robot_params.goal(1), robot_params.goal(2), ...
    cos(robot_params.goal(3)), sin(robot_params.goal(3)), 'k');
qrg.Marker = 'o';
qrg.MarkerFaceColor = 'k';
% plot human goal g1.
orange_c = [1,0.5,0];
scatter(robot_params.g1(1), robot_params.g1(2), 'markerfacecolor', orange_c, 'markeredgecolor', orange_c);
text(robot_params.g1(1), robot_params.g1(2)+0.5, 'g1', 'color', orange_c);
% plot human goal g2.
scatter(robot_params.g2(1), robot_params.g2(2), 'markerfacecolor', 'b', 'markeredgecolor', 'b');
text(robot_params.g2(1)-0.5, robot_params.g2(2), 'g2',  'color', 'b');
% plot human state
hsh = quiver(human_traj{1}(1), human_traj{2}(1), cos(human_traj{5}(1)), sin(human_traj{5}(1)), 'r', 'filled');
hsh.Marker = 'o';
hsh.MarkerFaceColor = 'r';
% plot robot state
rsh = quiver(r_plan{1}{1}(1), r_plan{1}{2}(1), cos(r_plan{1}{5}(1)), sin(r_plan{1}{5}(1)), 'k', 'filled');
rsh.Marker = 'o';
rsh.MarkerFaceColor = 'k';
    
% Plot obstacles.
%contour(robot_params.g2d.xs{1}, robot_params.g2d.xs{2}, robot_params.raw_sd_obs, [-0.1,0.1], ...
%    'Color', [0.4,0.4,0.4], 'LineStyle', '-', 'LineWidth', 2);
for oi=1:length(robot_params.obstacles)
    obs_info = robot_params.obstacles{oi};
    obs_min = obs_info(1:2);

    x_min = obs_min(1);
    y_min = obs_min(2);
    rectangle('position', obs_info, ...
                'curvature', [0.2,0.2], ...
                'EdgeColor', 'k');
end

% plot human preds
g1_th = scatter(g1_preds{1}, g1_preds{2}, 'markerfacecolor', 'none', 'markeredgecolor', orange_c);
g2_th = scatter(g2_preds{1}, g2_preds{2}, 'markerfacecolor', 'none', 'markeredgecolor', 'b');

% Plot robot shared plan, and contingency plans.
rps_h = scatter(r_plan{1}{1}, r_plan{1}{2}, 'markeredgecolor', 'k');
rpg1_h = scatter(r_plan{2}{1}, r_plan{2}{2}, 'markeredgecolor', orange_c);
rpg2_h = scatter(r_plan{3}{1}, r_plan{3}{2}, 'markeredgecolor', 'b');

%% tmp.

%% TODO: NEED TO DO THIS RIGHT WITH CREATING THE FINAL TRAJ THE ROBOT AND HUMAN FOLLOWS

coll_t_idx = 20;

hsh = quiver(human_traj{1}(coll_t_idx), ...
             human_traj{2}(coll_t_idx), ...
             cos(human_traj{5}(coll_t_idx)), ...
             sin(human_traj{5}(coll_t_idx)), 'r', 'filled');
hsh.Marker = 'o';
hsh.MarkerFaceColor = 'r';
rsh = quiver(r_plan{3}{1}(coll_t_idx), ...
             r_plan{3}{2}(coll_t_idx), ...
             cos(r_plan{3}{5}(coll_t_idx)), ...
             sin(r_plan{3}{5}(coll_t_idx)), 'k', 'filled');
rsh.Marker = 'o';
rsh.MarkerFaceColor = 'k';

pos = [human_traj{1}(coll_t_idx)-robot_params.car_rad, ...
        human_traj{2}(coll_t_idx)-robot_params.car_rad, ...
        robot_params.car_rad*2, robot_params.car_rad*2];
hrecth = rectangle('position', pos, ...
            'curvature', [1,1], ...
            'EdgeColor', 'r');
pos = [r_plan{3}{1}(coll_t_idx)-robot_params.car_rad, ...
        r_plan{3}{2}(coll_t_idx)-robot_params.car_rad, ...
        robot_params.car_rad*2, robot_params.car_rad*2];
rrecth = rectangle('position', pos, ...
            'curvature', [1,1], ...
            'EdgeColor', 'k');
            

%% Set limits.
xlim([robot_params.gmin(1), robot_params.gmax(1)])
ylim([robot_params.gmin(2), robot_params.gmax(2)])
% Plot the lane boundaries.
plot([robot_params.gmin(1), -4.5], [0,0], '--y', 'linewidth', 4);
plot([4.5, robot_params.gmax(1)], [0,0], '--y', 'linewidth', 4);
plot([0,0], [robot_params.gmin(2), -4.5], '--y', 'linewidth', 4);
plot([0,0], [4.5, robot_params.gmax(2)], '--y', 'linewidth', 4);
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])
box on
set(gca,'xtick',[])
set(gca,'xticklabel',[])
set(gca,'ytick',[])
set(gca,'yticklabel',[])

clear all
close all

%% Load up all the info for robot.
robot_params = exp3_planner_baselines();

%% Initial human state
h_x0 = [6, 1.83, pi, 0.01]; 
true_human_goal = 'g1'; % pick which goal the simulated human is going to.
%true_human_goal = 'g2';

%% Setup robot start state. 
r_start = [-6, -1.83, 0, 0.01]; 

%% Video creation!
save_video = false;
if save_video
    vout = VideoWriter(strcat('exp3_safe_baseline_',true_human_goal,'_',datestr(now,'HH_MM_SS_FFF'),'.mp4'),'MPEG-4');
    vout.Quality = 100;
    vout.FrameRate = 5;
    vout.open;
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
hsh = quiver(h_x0(1), h_x0(2), cos(h_x0(3)), sin(h_x0(3)), 'r', 'filled');
hsh.Marker = 'o';
hsh.MarkerFaceColor = 'r';
% plot robot state
rsh = quiver(r_start(1), r_start(2), cos(r_start(3)), sin(r_start(3)), 'k', 'filled');
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

% Set limits.
xlim([robot_params.gmin(1), robot_params.gmax(1)])
ylim([robot_params.gmin(2), robot_params.gmax(2)])
% Plot the lane boundaries.
plot([robot_params.gmin(1), -4.5], [0,0], '--y', 'linewidth', 4);
plot([4.5, robot_params.gmax(1)], [0,0], '--y', 'linewidth', 4);
plot([0,0], [robot_params.gmin(2), -4.5], '--y', 'linewidth', 4);
plot([0,0], [4.5, robot_params.gmax(2)], '--y', 'linewidth', 4);
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])

%% Setup simulation horizon. 
h_xcurr = h_x0;
r_xcurr = r_start;

pgoals = robot_params.goal_prior; 

gray_c = [0.8,0.8,0.8];

rph = [];
g1_th = [];
g2_th = [];
hrecth = [];
rrecth = [];

% Predict human!
fprintf('Predicting g1...\n');
human_preds_g1 = ...
        robot_params.predictor_g1.plan(h_xcurr, robot_params.g1, [], []);

fprintf('Predicting g2...\n');
human_preds_g2 = ...
        robot_params.predictor_g2.plan(h_xcurr, robot_params.g2, [], []);   

% Plan for the robot!
fprintf('Planning...\n');
robot_plan = ...
    robot_params.planner.plan(r_xcurr, robot_params.goal, ...
                                human_preds_g1, human_preds_g2);

simT = length(robot_plan{1}); %60;

for t=1:simT
    % Update human state based on the true goal they are going to.
    if strcmp(true_human_goal, 'g1')
        h_xnext = [human_preds_g1{1}(t), human_preds_g1{2}(t), human_preds_g1{5}(t), human_preds_g1{3}(t)];
    elseif strcmp(true_human_goal, 'g2')
        h_xnext = [human_preds_g2{1}(t), human_preds_g2{2}(t), human_preds_g2{5}(t), human_preds_g2{3}(t)];
    else
        error('invalid true human goal!');
    end
    
    % Update robot state.
    r_xnext = [robot_plan{1}(t), robot_plan{2}(t), robot_plan{5}(t), robot_plan{3}(t)];
    
    %% Plot robot plan
    hsh.Color = gray_c;
    hsh.MarkerFaceColor = gray_c;
    rsh.Color = gray_c;
    rsh.MarkerFaceColor = gray_c;
    if ~isempty(rph)
        rph.CData = gray_c;
    end
    if ~isempty(g1_th) && ~isempty(g2_th)
        delete(g1_th);
        delete(g2_th);
        delete(hrecth);
        delete(rrecth);
        %g1_th.MarkerEdgeColor = gray_c;
        %g2_th.MarkerEdgeColor = gray_c;
    end

    % plot human preds
    g1_th = scatter(human_preds_g1{1}, human_preds_g1{2}, 'markerfacecolor', 'none', 'markeredgecolor', orange_c);
    g2_th = scatter(human_preds_g2{1}, human_preds_g2{2}, 'markerfacecolor', 'none', 'markeredgecolor', 'b');
    % plot robot plan
    rph = scatter(robot_plan{1}, robot_plan{2}, 'k');
    
    %hsh = scatter(h_xcurr(1), h_xcurr(2), 'r', 'filled');
    %rsh = scatter(r_xcurr(1), r_xcurr(2), 'k', 'filled');
    hsh = quiver(h_xcurr(1), h_xcurr(2), cos(h_xcurr(3)), sin(h_xcurr(3)), 'r', 'filled');
    hsh.Marker = 'o';
    hsh.MarkerFaceColor = 'r';
    rsh = quiver(r_xcurr(1), r_xcurr(2), cos(r_xcurr(3)), sin(r_xcurr(3)), 'k', 'filled');
    rsh.Marker = 'o';
    rsh.MarkerFaceColor = 'k';
    
    pos = [h_xcurr(1)-robot_params.car_rad, h_xcurr(2)-robot_params.car_rad, ...
           robot_params.car_rad*2, robot_params.car_rad*2];
    hrecth = rectangle('position', pos, ...
                'curvature', [1,1], ...
                'EdgeColor', 'r');
    pos = [r_xcurr(1)-robot_params.car_rad, r_xcurr(2)-robot_params.car_rad, ...
           robot_params.car_rad*2, robot_params.car_rad*2];
    rrecth = rectangle('position', pos, ...
                'curvature', [1,1], ...
                'EdgeColor', 'k');
    
    % Plot the belief.
%     if strcmp(coll_check, 'conf')
%         if ~isempty(tb1)
%             delete(tb1)
%             delete(tb2)
%         end
%         tb1 = text(-4.7,-1,strcat('b_', num2str(t-1),'(low conf) = ', num2str(pbeta(1))));
%         tb1.Color = 'r';
%         tb2 = text(-4.7,-1.5,strcat('b_', num2str(t-1),'(high conf) = ', num2str(pbeta(2))));
%         tb2.Color = 'r';
%     end

    if save_video
        current_frame = getframe(gcf); %gca does just the plot
        writeVideo(vout,current_frame);
    end
    pause(0.1)
    
    %% Update belief over goals.
    %if strcmp(coll_check, 'conf')
    %    pbeta = human_params.predictor.belief_update(h_xnext, h_xcurr, pbeta);
    %end
    
    %% Finally update states.
    h_xcurr = h_xnext; 
    r_xcurr = r_xnext;
end
if save_video
    vout.close()
end
clear all
close all

%% Load up all the info for robot.
robot_params = exp3_contingency_planner();

%% Initial human state
h_x0 = [5.6, 2.25, pi, 0.01]; 
%true_human_goal = 'g1'; % pick which goal the simulated human is going to.
true_human_goal = 'g2';

%% Setup robot start state. 
r_start = [-5.6, -2.25, 0, 0.01]; 

%% Video creation!
save_video = false;
if save_video
    curr_date = datestr(now,'mm_dd_yyyy_HH_MM');
    filename = strcat('exp3_contingency_',true_human_goal,...
                        '_b(straight)',num2str(robot_params.goal_prior(1)), ...
                        '_', curr_date,'.mp4');
    vout = VideoWriter(filename,'MPEG-4');
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
contour(robot_params.g2d.xs{1}, robot_params.g2d.xs{2}, robot_params.raw_sd_obs, [0,0.1], ...
    'Color', [0.4,0.4,0.4], 'LineStyle', '-', 'LineWidth', 2);
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
simT = 50;
h_xcurr = h_x0;
r_xcurr = r_start;

% Setup belief
pgoals = robot_params.goal_prior;

% Setup branching time. 
branch_t = 1.38;

%% colors n stuff
gray_c = [0.8,0.8,0.8];

rps_h = [];
rpg1_h = [];
rpg2_h = [];
g1_th = [];
g2_th = [];
tb1 = [];
tb2 = [];

%% DO STUFF!
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
    robot_params.planner.contingency_plan(r_xcurr, robot_params.goal, ...
                                human_preds_g1, human_preds_g2, pgoals, branch_t);
shared_plan = robot_plan{1};
g1_plan = robot_plan{2};
g2_plan = robot_plan{3};
         
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
    if t < length(shared_plan{1})
        r_xnext = [shared_plan{1}(t+1), shared_plan{2}(t+1), shared_plan{5}(t+1), shared_plan{3}(t+1)];
    else
        tidx = t - length(shared_plan{1}) + 1;
        if strcmp(true_human_goal, 'g1')
            r_xnext = [g1_plan{1}(tidx), g1_plan{2}(tidx), g1_plan{5}(tidx), g1_plan{3}(tidx)];
        else
            r_xnext = [g2_plan{1}(tidx), g2_plan{2}(tidx), g2_plan{5}(tidx), g2_plan{3}(tidx)];
        end
    end
    
    
    %% Plot robot plan
    hsh.Color = gray_c;
    hsh.MarkerFaceColor = gray_c;
    rsh.Color = gray_c;
    rsh.MarkerFaceColor = gray_c;
    if ~isempty(rps_h)
        rps_h.CData = gray_c;
        rpg1_h.CData = gray_c;
        rpg2_h.CData = gray_c;
    end
    if ~isempty(g1_th) && ~isempty(g2_th)
        delete(g1_th);
        delete(g2_th);
    end
    % plot human preds
    g1_th = scatter(human_preds_g1{1}, human_preds_g1{2}, 'markerfacecolor', 'none', 'markeredgecolor', orange_c);
    g2_th = scatter(human_preds_g2{1}, human_preds_g2{2}, 'markerfacecolor', 'none', 'markeredgecolor', 'b');
    
    % Plot robot shared plan, and contingency plans.
    rps_h = scatter(shared_plan{1}, shared_plan{2}, 'markeredgecolor', 'k');
    rpg1_h = scatter(g1_plan{1}, g1_plan{2}, 'markeredgecolor', orange_c);
    rpg2_h = scatter(g2_plan{1}, g2_plan{2}, 'markeredgecolor', 'b');
    
    % Plot current human state.
    hsh = quiver(h_xcurr(1), h_xcurr(2), cos(h_xcurr(3)), sin(h_xcurr(3)), 'r', 'filled');
    hsh.Marker = 'o';
    hsh.MarkerFaceColor = 'r';
    % Plot current robot state.
    rsh = quiver(r_xcurr(1), r_xcurr(2), cos(r_xcurr(3)), sin(r_xcurr(3)), 'k', 'filled');
    rsh.Marker = 'o';
    rsh.MarkerFaceColor = 'k';
    
    % Plot the belief.
    if ~isempty(tb1)
        delete(tb1)
        delete(tb2)
    end
    tb1 = text(-4.7,-1,strcat('b_{', num2str(t-1),'}(straight) = ', num2str(pgoals(1))));
    tb1.Color = orange_c;
    tb2 = text(-4.7,-1.5,strcat('b_{', num2str(t-1),'}(left) = ', num2str(pgoals(2))));
    tb2.Color = 'b';

    if save_video
        % Write the video.
        current_frame = getframe(gcf); %gca does just the plot
        writeVideo(vout,current_frame);
    end
    pause(0.1)
    
    %% Update belief over goals.
    %pbeta = human_params.predictor.belief_update(h_xnext, h_xcurr, pgoals);
    
    %% Finally update states.
    h_xcurr = h_xnext; 
    r_xcurr = r_xnext;
end

if save_video
    vout.close()
end
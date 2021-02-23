clear all
close all

%% Choose which planner type to visualize.
% 'safe', 'heuristic', 'maxttl'
planner_type = 'safe';
%planner_type = 'heuristic';
%planner_type = 'maxttl';

if strcmp(planner_type, 'safe')
    load('exp_3_safe_baseline_trajs.mat');
elseif strcmp(planner_type, 'heuristic')
    %load('exp_3_heuristic_baseline_trajs_brancht0.3265.mat');
    
    % == final data for SIMPLE model branching time with NEW COST FUNCTION == %
    load('NEWCOSTFUN_exp_3_heuristic_baseline_trajs_brancht0.3265.mat');
    % ======================================================================= %
    
elseif strcmp(planner_type, 'maxttl')
    %load('exp_3_contingency_planner_uthresh0.33.mat');
    %load('exp_3_contingency_planner_uthresh0.27.mat');
    
    % == final data for SIMPLE model branching time with NEW COST FUNCTION == %
    load('fine_exp_3_contingency_planner_uthresh0.27.mat');
    % ======================================================================= %
    
    %load('complex_exp_3_contingency_planner_uthresh0.4.mat');
    %load('NEWCOSTFUN_complex_exp_3_contingency_planner_uthresh0.35.mat');
    %load('NEWCOSTFUN_LARGE_complex_exp_3_contingency_planner_uthresh0.35.mat');
    %load('NEWCOSTFUN_LARGE2_complex_exp_3_contingency_planner_uthresh0.35.mat');
else
    error('unsupported planner type');
end

%% Colors.
g1_color = [255, 0, 20]/255.; %[255, 36, 36]/255.; %[184, 0, 18]/255.; 
g2_color = [36, 135, 255]/255.; % [0, 116, 184]/255.; 
green_c = [75, 148, 95]/255.;
bg_color = [0.2,0.2,0.2]; % [0.6,0.6,0.6];
orange_c = [1,0.5,0];
safe_color = [1,1,1];

%% Choose which experiment to visualize
%--> ri = 1, hi = 3, pgi = 1, gi = 1
%--> sim_idx = 7
%gi = 1;
%i = 7;
%pgi = 1;

%--> ri = 1, hi = 1, pgi = 1, gi = 1
% gi = 1;
% i = 1;
% pgi = 1;

%--> ri = 2, hi = 2, pgi = 1, gi = 1
% ri = 2; hi = 2; pgi = 1; gi = 1;
% i = 13;

%-->ri = 2, hi = 2, pgi = 1, gi = 1
% ri = 1; hi = 3; pgi = 2; gi = 2;
% i = 8;

% ==== NICE EXAMPLE FOR SIMPLE SCENARIO ==== %
ri = 2; hi = 1; pgi = 1; gi = 1;
i = 10;
if strcmp(planner_type, 'safe')
    i = ri*hi; 
end
% ==== NICE EXAMPLE FOR SIMPLE SCENARIO ==== %

goal = all_h_goals{gi};
g1_preds = all_g1_preds{i};
g2_preds = all_g2_preds{i};
r_plan = all_plans{i};

if strcmp(goal, 'g1')
    human_traj = g1_preds;
    human_color = g1_color;
else
    human_traj = g2_preds;
    human_color = g2_color;
end

%% Plot the initial setup.
figure(1)
hold on
% plot robot goal.
qrg = quiver(robot_params.goal(1), ...
            robot_params.goal(2), ...
            cos(robot_params.goal(3)), ...
            sin(robot_params.goal(3)), 1, 'filled');
qrg.Marker = 'o';
qrg.MarkerFaceColor = bg_color;
qrg.MarkerEdgeColor = safe_color;
qrg.Color = safe_color;
qrg.LineWidth = 1.5;

% plot human goal g1.
hg1 = quiver(robot_params.g1(1), ...
         robot_params.g1(2), ...
         cos(robot_params.g1(3)), ...
         sin(robot_params.g1(3)), 1, 'filled');
hg1.Marker = 'o';
hg1.MarkerFaceColor = bg_color;
hg1.MarkerEdgeColor = g1_color;
hg1.Color = g1_color;
hg1.LineWidth = 1.5;
%text(robot_params.g1(1), robot_params.g1(2)+0.5, 'g1', 'color', g1_color);

% plot human goal g2.
hg1 = quiver(robot_params.g2(1), ...
         robot_params.g2(2), ...
         cos(robot_params.g2(3)), ...
         sin(robot_params.g2(3)), 1, 'filled');
hg1.Marker = 'o';
hg1.MarkerFaceColor = bg_color;
hg1.MarkerEdgeColor = g2_color;
hg1.Color = g2_color;
hg1.LineWidth = 1.5;

%text(robot_params.g2(1)-0.5, robot_params.g2(2), 'g2',  'color', g2_color);
% % plot human state
% hsh = quiver(human_traj{1}(1), human_traj{2}(1), cos(human_traj{5}(1)), sin(human_traj{5}(1)), 'r', 'filled');
% hsh.Marker = 'o';
% hsh.MarkerFaceColor = 'r';
% % plot robot state
% rsh = quiver(r_plan{1}(1), r_plan{2}(1), cos(r_plan{5}(1)), sin(r_plan{5}(1)), 'k', 'filled');
% rsh.Marker = 'o';
% rsh.MarkerFaceColor = 'k';
    
%% Plot obstacles.
%contour(robot_params.g2d.xs{1}, robot_params.g2d.xs{2}, robot_params.raw_sd_obs, [-0.1,0.1], ...
%    'Color', [0.4,0.4,0.4], 'LineStyle', '-', 'LineWidth', 2);
for oi=1:length(robot_params.obstacles)
    obs_info = robot_params.obstacles{oi};
    obs_min = obs_info(1:2);

    x_min = obs_min(1);
    y_min = obs_min(2);
    rectangle('position', obs_info, ...
                'curvature', [0.1,0.1], ...
                'EdgeColor', 'k', ...
                'FaceColor', green_c);
end

%% Plot human preds
new_num_waypts = 20;
g1_start_c = [255, 105, 117]/255.;
g1_end_c = g1_color;
g1_traj_c = create_color_arr(g1_start_c, g1_end_c, new_num_waypts);

g2_start_c = [102, 199, 255]/255.;
g2_end_c = g2_color;
g2_traj_c = create_color_arr(g2_start_c, g2_end_c, new_num_waypts);

[g1_xdownsamp, g1_ydownsamp] = ...
    downsample_traj(g1_preds{1}, g1_preds{2}, robot_params.num_waypts, new_num_waypts);
[g2_xdownsamp, g2_ydownsamp] = ...
    downsample_traj(g2_preds{1}, g2_preds{2}, robot_params.num_waypts, new_num_waypts);

for ti=2:new_num_waypts
    g1_th = plot([g1_xdownsamp(ti), g1_xdownsamp(ti-1)], ...
                [g1_ydownsamp(ti), g1_ydownsamp(ti-1)], '-o');
    g1_th.MarkerFaceColor = bg_color; 
    g1_th.MarkerEdgeColor = g1_traj_c(ti, :);
    g1_th.Color = g1_traj_c(ti, :);
    g1_th.LineWidth = 2;
    
    g2_th = plot([g2_xdownsamp(ti), g2_xdownsamp(ti-1)], ...
                [g2_ydownsamp(ti), g2_ydownsamp(ti-1)], '-o');
    g2_th.MarkerFaceColor = bg_color; 
    g2_th.MarkerEdgeColor = g2_traj_c(ti, :);
    g2_th.Color = g2_traj_c(ti, :);
    g2_th.LineWidth = 2;
end

% Future timestep to visualize.
future_t_idx = 20;

if strcmp(planner_type, 'safe')
    color = 'k';
    
    % Plot the safe plan
    [xdownsamp, ydownsamp] = ...
        downsample_traj(r_plan{1}, r_plan{2}, robot_params.num_waypts, new_num_waypts);
    
    % Setup colors for robot traj.
    start_c = [120, 120, 120]/255.;
    end_c = safe_color;
    safe_traj_c = create_color_arr(start_c, end_c, new_num_waypts);
    
    for ti=2:new_num_waypts
        rph = plot([xdownsamp(ti), xdownsamp(ti-1)], ...
                    [ydownsamp(ti), ydownsamp(ti-1)], '-o');
        rph.MarkerFaceColor = bg_color; %safe_traj_c(ti, :);
        rph.MarkerEdgeColor = safe_traj_c(ti, :);
        rph.Color = safe_traj_c(ti, :);
        rph.LineWidth = 2;
    end
    
    idx = 1:robot_params.num_waypts;
    idxq = linspace(min(idx), max(idx), new_num_waypts);
    thdownsamp = interp1(idx, r_plan{5}, idxq, 'linear');
    r_traj = [xdownsamp', ydownsamp', thdownsamp'];
    
else
    % Setup measurement frequency in (s)
    measurement_freq = 0.0891;
    robot_dt = robot_params.dt;
    num_obs_per_dt = floor(robot_dt/measurement_freq);

    % Create the belief updater. 
    load('belief_updater.mat');

    shared_spline = r_plan{1};
    g1_spline = r_plan{2};
    g2_spline = r_plan{3};
    pgoals = all_priors{pgi};
    if strcmp(planner_type, 'maxttl')
        branch_t = all_branch_t{i};
    end
    
    % Plot robot shared plan, and contingency plans.

    % Plot the safe plan
    num_shared_pts = length(shared_spline{1});
    xdownsamp = shared_spline{1};
    ydownsamp = shared_spline{2}; % no downsampling!
    
    % Setup colors for robot traj.
    start_c = [120, 120, 120]/255.;
    end_c = safe_color;
    safe_traj_c = create_color_arr(start_c, end_c, num_shared_pts);
    
    for ti=2:num_shared_pts
        rph = plot([xdownsamp(ti), xdownsamp(ti-1)], ...
                    [ydownsamp(ti), ydownsamp(ti-1)], '-o');
        rph.MarkerFaceColor = bg_color; %safe_traj_c(ti, :);
        rph.MarkerEdgeColor = safe_traj_c(ti, :);
        rph.Color = safe_traj_c(ti, :);
        rph.LineWidth = 2;
    end
    
    num_contingency_pts = length(g1_spline{1});
    new_num_cont_pts = round(num_contingency_pts/2);
    [g1_xdownsamp, g1_ydownsamp] = ...
        downsample_traj(g1_spline{1}, g1_spline{2}, ...
        num_contingency_pts, new_num_cont_pts);
    [g2_xdownsamp, g2_ydownsamp] = ...
        downsample_traj(g2_spline{1}, g2_spline{2}, ...
        num_contingency_pts, new_num_cont_pts);

    g1_new_start_c = g1_traj_c(num_shared_pts, :);
    g2_new_start_c = g2_traj_c(num_shared_pts, :);
    g1_traj_c = create_color_arr(g1_new_start_c, g1_end_c, new_num_cont_pts);
    g2_traj_c = create_color_arr(g2_new_start_c, g2_end_c, new_num_cont_pts);
    
    for ti=2:new_num_cont_pts
        g1_th = plot([g1_xdownsamp(ti), g1_xdownsamp(ti-1)], ...
                    [g1_ydownsamp(ti), g1_ydownsamp(ti-1)], '-o');
        g1_th.MarkerFaceColor = bg_color; 
        g1_th.MarkerEdgeColor = g1_traj_c(ti, :);
        g1_th.Color = g1_traj_c(ti, :);
        g1_th.LineWidth = 2;

        g2_th = plot([g2_xdownsamp(ti), g2_xdownsamp(ti-1)], ...
                    [g2_ydownsamp(ti), g2_ydownsamp(ti-1)], '-o');
        g2_th.MarkerFaceColor = bg_color; 
        g2_th.MarkerEdgeColor = g2_traj_c(ti, :);
        g2_th.Color = g2_traj_c(ti, :);
        g2_th.LineWidth = 2;
    end    

    % Compute what posterior would be after branch_t timesteps.
    posterior = compute_posterior(human_traj, pgoals, ...
                        robot_params.planner.times, branch_t, ...
                        num_obs_per_dt, measurement_freq, ...
                        gnums, belief_updater);
    [maxval, max_gidx] = max(posterior);
    
    % Pick the trajectory that we end up committing to. 
    if max_gidx == 1
        branch_traj = g1_spline;
    elseif max_gidx == 2 
        branch_traj = g2_spline;
    end
    
    xs_shared = shared_spline{1};
    ys_shared = shared_spline{2};
    ths_shared = shared_spline{5};
    xs_branch = branch_traj{1}(2:end); % don't include shared point that is repeated. 
    ys_branch = branch_traj{2}(2:end);
    ths_branch = branch_traj{5}(2:end);
    r_traj = [xs_shared', ys_shared', ths_shared'];
    r_traj = vertcat(r_traj, [xs_branch', ys_branch', ths_branch']);
    
%     % Plot future state of robot car. 
%     rsh = quiver(r_traj(future_t_idx, 1), ...
%                  r_traj(future_t_idx, 2), ...
%                  cos(r_traj(future_t_idx, 3)), ...
%                  sin(r_traj(future_t_idx, 3)), 1, 'filled');
%     rsh.Marker = 'o';
%     rsh.MarkerFaceColor = safe_color;
%     rsh.MarkerEdgeColor = safe_color;
%     rsh.Color = safe_color;
%     rsh.LineWidth = 2;
    
%     % Plot the body of the robot car. 
%     pos = [r_traj(future_t_idx, 1)-robot_params.car_rad, ...
%             r_traj(future_t_idx, 2)-robot_params.car_rad, ...
%             robot_params.car_rad*2, robot_params.car_rad*2];
%     rrecth = rectangle('position', pos, ...
%                 'curvature', [1,1], ...
%                 'EdgeColor', 'k');
        
end

% % Plot the future state of the human. 
% hsh = quiver(human_traj{1}(future_t_idx), ...
%              human_traj{2}(future_t_idx), ...
%              cos(human_traj{5}(future_t_idx)), ...
%              sin(human_traj{5}(future_t_idx)), 1, 'filled');
% hsh.Marker = 'o';
% hsh.MarkerFaceColor = human_color;
% hsh.MarkerEdgeColor = human_color;
% hsh.Color = human_color;
% hsh.LineWidth = 2;

% % Plot body of the human vehicle.
% pos = [human_traj{1}(future_t_idx)-robot_params.car_rad, ...
%         human_traj{2}(future_t_idx)-robot_params.car_rad, ...
%         robot_params.car_rad*2, robot_params.car_rad*2];
% hrecth = rectangle('position', pos, ...
%             'curvature', [1,1], ...
%             'EdgeColor', human_color);

%% Set limits.
xlim([robot_params.gmin(1), robot_params.gmax(1)])
ylim([robot_params.gmin(2), robot_params.gmax(2)])
% Plot the lane boundaries.
lane_c = [255, 196, 36]/255.;
plot([robot_params.gmin(1), -4.5], [0,0], '--', 'color', lane_c, 'linewidth', 4);
plot([4.5, robot_params.gmax(1)], [0,0], '--y', 'color', lane_c,'linewidth', 4);
plot([0,0], [robot_params.gmin(2), -4.5], '--y', 'color', lane_c,'linewidth', 4);
plot([0,0], [4.5, robot_params.gmax(2)], '--y', 'color', lane_c,'linewidth', 4);
set(gcf, 'color', 'k')
set(gcf, 'position', [0,0,900,900])
box on
set(gca,'xtick',[])
set(gca,'xticklabel',[])
set(gca,'ytick',[])
set(gca,'yticklabel',[])
set(gca,'Color',bg_color)


%% =================== Simulate the trajectories! =================== %%
v = VideoWriter(strcat(planner_type,'_planner.mp4'),'MPEG-4');
v.Quality = 100;
open(v)

for i=1:robot_params.num_waypts
    % Plot EGO car
    qr = quiver(r_traj(i,1), r_traj(i,2), ...
                cos(r_traj(i,3)), sin(r_traj(i,3)), ...
                'Color', 'w');

    % Plot body of the EGO vehicle. 
    pos = [r_traj(i,1)-robot_params.car_rad, r_traj(i,2)-robot_params.car_rad, ...
           robot_params.car_rad*2, robot_params.car_rad*2];
    rr = rectangle('position', pos, ...
                'curvature', [1,1], ...
                'EdgeColor', 'w');
    
    qr.ShowArrowHead = 'on';
    qr.AutoScale = 'off';
    qr.AutoScaleFactor = 0.5;
    qr.Marker = 'o';
    qr.MarkerFaceColor = 'w';
    
    % Plot car going to g1.
    qh = quiver(human_traj{1}(i), human_traj{2}(i), ...
                cos(human_traj{5}(i)), sin(human_traj{5}(i)), ...
                'Color', 'r');
    qh.ShowArrowHead = 'on';
    qh.AutoScale = 'off';
    qh.AutoScaleFactor = 0.5;  
    qh.Marker = 'o';
    qh.MarkerFaceColor = 'r';
    
    pos = [human_traj{1}(i)-robot_params.car_rad, human_traj{2}(i)-robot_params.car_rad, ...
           robot_params.car_rad*2, robot_params.car_rad*2];
    rh = rectangle('position', pos, ...
                'curvature', [1,1], ...
                'EdgeColor', 'r');
    
    %% write the video files
    frame = getframe(gcf);
    writeVideo(v,frame);

    pause(0.1);
    
    delete(qr);
    delete(qh);
    delete(rr);
    delete(rh);
    
end

close(v)

%% ======================== HELPER FUNCS ============================= %%

%% Plots a single safe trajectory
function rph = plot_safe_traj(r_plan, color)
    % plot robot plan
    rph = scatter(r_plan{1}, r_plan{2}, 'markeredgecolor', color);
end

%% Compute the belief after branch_t observations.
function posterior = compute_posterior(human_traj, prior, times, ...
                                        branch_t, num_obs_per_dt, ...
                                        measurement_freq, gnums, belief_updater)
    % Compute number of waypoints in each part of the plan.
    [~,end_idx] = min(abs(times - branch_t));
    shared_num_waypts = end_idx;
    
    % Should we remove the last timestep because at the last timestep we 
    % need to commit to one of the two branched trajectories?
    shared_num_waypts = shared_num_waypts-1;
  
    % NOTE: THIS ASSUMES WE GET ONE OBSERVATION DURING EACH ROBOT TIMESTEP!
    posterior = prior;
    for t=2:shared_num_waypts
        h_xcurr = [human_traj{1}(t-1), human_traj{2}(t-1), human_traj{5}(t-1)];
        h_xnext = [human_traj{1}(t), human_traj{2}(t), human_traj{5}(t)];
        
        % Invert the dynamics to find the action that was probably taken. 
        uidx = belief_updater.inv_dyn(h_xnext, h_xcurr);
        
        % Create joint state!
        z = {h_xcurr(1), h_xcurr(2), h_xcurr(3), posterior(1)};
        
        % Update posterior!
        posterior_g1 = belief_updater.belief_update(uidx,z);
        posterior = [posterior_g1, 1-posterior_g1];
    end
end

%% util function which creates rgb array based on colors 
function color_arr = create_color_arr(start_c, end_c, num_pts)
    rc = linspace(start_c(1), end_c(1), num_pts);
    gc = linspace(start_c(2), end_c(2), num_pts);
    bc = linspace(start_c(3), end_c(3), num_pts);
    color_arr = [rc', gc', bc'];
end

%% util function to downsample traj
function [xdownsamp, ydownsamp] = downsample_traj(xs, ys, ...
    original_num_pts, new_num_pts)
    idx = 1:original_num_pts;
    idxq = linspace(min(idx), max(idx), new_num_pts);
    xdownsamp = interp1(idx, xs, idxq, 'linear');
    ydownsamp = interp1(idx, ys, idxq, 'linear');
end
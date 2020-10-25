clear all
close all

%load('exp_3_contingency_planner_uthresh0.33.mat');
load('new_exp_3_contingency_planner_uthresh0.33.mat');

%% Save out important metrics.
all_d_to_goal_5050 = [];
all_d_to_human_5050 = [];

all_d_to_goal_correct = [];
all_d_to_human_correct = [];

all_d_to_goal_incorrect = [];
all_d_to_human_incorrect = [];

% Setup measurement frequency in (s)
measurement_freq = 0.0891;
robot_dt = robot_params.dt;
num_obs_per_dt = floor(robot_dt/measurement_freq);

% Create the belief updater. 
load('belief_updater.mat');
% gmin = [-7.75, -7.75, 0, 0]; 
% gmax = [7.75, 7.75, 2*pi, 1]; 
% gnums = [30, 30, 20, 20];
% thetas = {[-6.5, 1.83, pi], [-1.83, -6.5, 3*pi/2]};
% gdisc = (gmax - gmin) ./ (gnums - 1);
% dt = gdisc(1)/robot_params.max_linear_vel;
% belief_updater = exp_3_belief_updater(gmin, gmax, gnums, ...
%                                         thetas, robot_params.max_linear_vel, ...
%                                         dt);
% === Branch times for uThresh = 0.33 === %
all_branch_times = containers.Map();
all_branch_times(num2str([6, 1.83, pi, 0.2])) = max(0.623563, 0.267241);
all_branch_times(num2str([6, 1.83, pi, 0.5])) = max(0.801724, 0.267241);
all_branch_times(num2str([6, 1.83, pi, 0.8])) = max(0.356322, 0.979885);
% ======================================= %

% % === Branch times for uThresh = 0.25 === %
% uThresh = 0.25;
% all_branch_times(num2str([6, 1.83, pi, 0.2])) = max(1.247126, 1.603448);
% all_branch_times(num2str([6, 1.83, pi, 0.5])) = max(1.247126, 1.514368);
% all_branch_times(num2str([6, 1.83, pi, 0.8])) = max(0.623563, 1.603448);
% % ======================================= %

% % === Branch times for uThresh = 0.27 === %
% uThresh = 0.27;
% all_branch_times(num2str([6, 1.83, pi, 0.2])) = max(0.623563, 1.425287);
% all_branch_times(num2str([6, 1.83, pi, 0.5])) = max(0.801724, 1.425287);
% all_branch_times(num2str([6, 1.83, pi, 0.8])) = max(0.356322, 1.425287);
% % ======================================= %

sim_idx = 1;
for ri = 1:length(all_r_x0s)
    for hi = 1:length(all_h_x0s)
        for pi = 1:length(all_priors)
            for gi=1:length(all_h_goals)
                goal = all_h_goals{gi};
                g1_preds = all_g1_preds{sim_idx};
                g2_preds = all_g2_preds{sim_idx};
                r_plan = all_plans{sim_idx};
                pgoals = all_priors{pi};     
                
                if strcmp(goal, 'g1')
                    human_traj = g1_preds;
                else
                    human_traj = g2_preds;
                end
                
                joint_state = [human_traj{1}(1), human_traj{2}(1), ...
                                human_traj{5}(1), pgoals(1)];
                branch_t = all_branch_times(num2str(joint_state));
                
                %% DEBUGGING MODE!
                if sim_idx == 9
                    shared_spline = r_plan{1};
                    spline_g1 = r_plan{2};
                    spline_g2 = r_plan{3};
                    
                    [~,end_idx] = min(abs(robot_params.planner.times - branch_t));
                    binned_branch_t = robot_params.planner.times(end_idx);
                    binned_horiz = robot_params.planner.times(end);
                    
                    num_contingency_waypts = length(spline_g2{1});
                    g2_start = [spline_g2{1}(1), spline_g2{2}(1), spline_g2{5}(1), spline_g2{3}(1)];
                    g2_end = [spline_g2{1}(end), spline_g2{2}(end), spline_g2{5}(end), spline_g2{3}(end)];
                    
                    %num_contingency_waypts = num_contingency_waypts*2;
                    %spline(g2_start, g2_end, binned_horiz-binned_branch_t, num_contingency_waypts);
                    total_spline2 = {cat(2,shared_spline{1},spline_g2{1}(2:end)), ...
                                 cat(2,shared_spline{2},spline_g2{2}(2:end)), ...
                                 cat(2,shared_spline{3},spline_g2{3}(2:end)), ...
                                 cat(2,shared_spline{4},spline_g2{4}(2:end)), ...
                                 cat(2,shared_spline{5},spline_g2{5}(2:end))};   
                    feasible_horizon2 = ...
                        robot_params.planner.compute_dyn_feasible_horizon(...
                                  total_spline2, ...
                                  robot_params.max_linear_vel, ...
                                  robot_params.max_angular_vel, ...
                                  binned_horiz-binned_branch_t);
                end
            end
            
            sim_idx = sim_idx + 1;
        end
    end
end


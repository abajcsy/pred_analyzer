clear all
close all

load('new_exp_3_contingency_planner_uthresh0.33.mat');

% === Branch times for uThresh = 0.33 === %
all_branch_times = containers.Map();
all_branch_times(num2str([6, 1.83, pi, 0.2])) = max(0.623563, 0.267241);
all_branch_times(num2str([6, 1.83, pi, 0.5])) = max(0.801724, 0.267241);
all_branch_times(num2str([6, 1.83, pi, 0.8])) = max(0.356322, 0.979885);
% ======================================= %

sim_idx = 9;
goal = 'g1';

g1_preds = all_g1_preds{sim_idx};
g2_preds = all_g2_preds{sim_idx};
r_plan = all_plans{sim_idx};
pgoals = [0.8,0.2];   
                              
if strcmp(goal, 'g1')
    human_traj = g1_preds;
else
    human_traj = g2_preds;
end

joint_state = [human_traj{1}(1), human_traj{2}(1), ...
                human_traj{5}(1), pgoals(1)];
branch_t = all_branch_times(num2str(joint_state));
                 

% Plan for the robot!
r_xcurr = [r_plan{1}{1}(1), r_plan{1}{2}(1), r_plan{1}{5}(1), r_plan{1}{3}(1)];
robot_plan = ...
    robot_params.planner.contingency_plan(r_xcurr, robot_params.goal, ...
                                g1_preds, g2_preds, pgoals, branch_t);
                            
quiver(robot_plan{3}{1}, robot_plan{3}{2}, ...
        cos(robot_plan{3}{5}), sin(robot_plan{3}{5}))
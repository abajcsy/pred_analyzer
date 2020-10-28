clear all
close all

%% Load up all the info for robot.
robot_params = exp3_contingency_planner();

%% Setup all robot start states. 
all_r_x0s = {[-6, -1.83, 0, 0.01], ...
             [-6, -1.83, 0, 1], ...
             [-6, -1.83, 0, 4]};
         
%% Setup all initial human states.
all_h_x0s = {[6, 1.83, pi, 0.01], ...
             [6, 1.83, pi, 1], ...
             [6, 1.83, pi, 4]};         

%% Setup all human goal states.
all_h_goals = {'g1', 'g2'};

%% Setup all priors over g1 (and thereby g2).
all_priors = {[0.2, 0.8], [0.5, 0.5], [0.8, 0.2]};

%% Setup heuristic branch time.
all_branch_times = containers.Map();

% % === Branch times for uThresh = 0.25 === %
% uThresh = 0.25;
% all_branch_times(num2str([6, 1.83, pi, 0.2])) = max(1.247126, 1.603448);
% all_branch_times(num2str([6, 1.83, pi, 0.5])) = max(1.247126, 1.514368);
% all_branch_times(num2str([6, 1.83, pi, 0.8])) = max(0.623563, 1.603448);
% % ======================================= %

% % === Branch times for uThresh = 0.27 === %
% uThresh = 0.27;
% all_branch_times(num2str([6, 1.83, pi, 0.2])) = max(0.712644, 1.336207);
% all_branch_times(num2str([6, 1.83, pi, 0.5])) = max(0.534483, 1.336207);
% all_branch_times(num2str([6, 1.83, pi, 0.8])) = max(0.267241, 1.336207);
% % ======================================= %

% === Branch times for uThresh = 0.33 === %
% this is basically safeguarding against opt policy!
% uThresh = 0.33;
% all_branch_times(num2str([6, 1.83, pi, 0.2])) = max(0.623563, 0.267241);
% all_branch_times(num2str([6, 1.83, pi, 0.5])) = max(0.801724, 0.267241);
% all_branch_times(num2str([6, 1.83, pi, 0.8])) = max(0.356322, 0.979885);
% ======================================= %


% === Branch times for uThresh = 0.35 === %
% THIS IS FOR THE FULL, TRUE MODEL!
uThresh = 0.35;
all_branch_times(num2str([6, 1.83, pi, 0.2])) = max(1.00, 2.00);
all_branch_times(num2str([6, 1.83, pi, 0.5])) = max(0.75, 2.00);
all_branch_times(num2str([6, 1.83, pi, 0.8])) = max(0.375, 2.00);
% ======================================= %

total_num_sims = length(all_r_x0s)*length(all_h_x0s)*length(all_priors);
sim_idx = 1;

%% Save all corresponding preds + planned trajectories
all_g1_preds = {};
all_g2_preds = {};
all_plans = {};
all_branch_t = {};

for ri = 1:length(all_r_x0s)
    for hi = 1:length(all_h_x0s)
        for pgi = 1:length(all_priors)
            r_xcurr = all_r_x0s{ri};
            h_xcurr = all_h_x0s{hi};
            pgoals = all_priors{pgi};
            joint_state = [h_xcurr(1), h_xcurr(2), h_xcurr(3), pgoals(1)];
            branch_t = all_branch_times(num2str(joint_state));
            
            fprintf('Running iteration %d / %d ...\n', sim_idx, total_num_sims);

            % Predict human!
            human_preds_g1 = ...
                    robot_params.predictor_g1.plan(h_xcurr, robot_params.g1, [], []);

            human_preds_g2 = ...
                    robot_params.predictor_g2.plan(h_xcurr, robot_params.g2, [], []);   

            % Plan for the robot!
            robot_plan = ...
                robot_params.planner.contingency_plan(r_xcurr, robot_params.goal, ...
                                            human_preds_g1, human_preds_g2, pgoals, branch_t);

            hold on
            quiver(robot_plan{1}{1}, robot_plan{1}{2}, cos(robot_plan{1}{5}), sin(robot_plan{1}{5}))
            quiver(robot_plan{2}{1}, robot_plan{2}{2}, cos(robot_plan{2}{5}), sin(robot_plan{2}{5}))
            quiver(robot_plan{3}{1}, robot_plan{3}{2}, cos(robot_plan{3}{5}), sin(robot_plan{3}{5}))
                                        
            % Save data!
            all_g1_preds{end+1} = human_preds_g1;
            all_g2_preds{end+1} = human_preds_g2;
            all_plans{end+1} = robot_plan;
            all_branch_t{end+1} = branch_t;

            sim_idx = sim_idx + 1;
        end
    end
end

save(strcat('fine_complex_exp_3_contingency_planner_uthresh',num2str(uThresh),'.mat'), ...
        'all_g1_preds', 'all_g2_preds', 'all_plans', ...
        'robot_params', 'all_r_x0s', 'all_h_x0s', 'all_h_goals', ...
        'all_priors', 'all_branch_t', 'robot_params');

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

total_num_sims = length(all_r_x0s)*length(all_h_x0s)*length(all_priors);
sim_idx = 1;

%% Save all corresponding preds + planned trajectories
all_g1_preds = {};
all_g2_preds = {};
all_plans = {};

% Setup heuristic branch time.
% heuristic branch times: 
%       0.5 s
%       robot_params.dt*2
branch_t = robot_params.dt*2; 

for ri = 1:length(all_r_x0s)
    for hi = 1:length(all_h_x0s)
        for pgi = 1:length(all_priors)
            r_xcurr = all_r_x0s{ri};
            h_xcurr = all_h_x0s{hi};
            pgoals = all_priors{pgi};
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

            % Save data!
            all_g1_preds{end+1} = human_preds_g1;
            all_g2_preds{end+1} = human_preds_g2;
            all_plans{end+1} = robot_plan;

            sim_idx = sim_idx + 1;
        end
    end
end

save('NEWCOSTFUN_exp_3_heuristic_baseline_trajs_2STEP_BRANCH.mat', ...
        'all_g1_preds', 'all_g2_preds', 'all_plans', ...
        'robot_params', 'all_r_x0s', 'all_h_x0s', 'all_h_goals', ...
        'all_priors', 'branch_t', 'robot_params');

clear all
close all

load('exp_3_safe_baseline_trajs.mat');

%% Save out important metrics.
all_d_to_goal = [];
all_d_to_human = [];

for i = 1:length(all_plans)
    for gi=1:length(all_h_goals)
        goal = all_h_goals{gi};
        g1_preds = all_g1_preds{i};
        g2_preds = all_g2_preds{i};
        r_plan = all_plans{i};
        
        if strcmp(goal, 'g1')
            human_traj = g1_preds;
        else
            human_traj = g2_preds;
        end

        % Compute dist between robot and goal. 
        d_to_goal = d_r_to_goal(r_plan, robot_params.goal);

        % Compute minimum distance between robot and human.
        min_d_to_human = min_d_between_r_and_h(human_traj, r_plan, ...
                                                robot_params.car_rad);

        % Store data.
        all_d_to_goal = [all_d_to_goal, d_to_goal];
        all_d_to_human = [all_d_to_human, min_d_to_human];
    end
end

% Avg of distance to goal.
avg_d_to_g = mean(all_d_to_goal);
std_d_to_g = std(all_d_to_goal);

% Avg of distance to human.
avg_d_to_h = mean(all_d_to_human);
std_d_to_h = std(all_d_to_human);

fprintf('============= SAFE BASELINE: ==========\n');
fprintf('--> average distance to goal: %f (m)\n', avg_d_to_g);
fprintf('    stdev distance to goal: %f (m)\n', std_d_to_g);
fprintf('--> average (min) distance to human: %f (m)\n', avg_d_to_h);
fprintf('    stdev (min) distance to human: %f (m)\n', std_d_to_h);
fprintf('=======================================\n');

%% SANITY CHECK!
%  plot the safe trajectories.
figure(1)
hold on
for i = 1:length(all_plans)
    r_plan = all_plans{i};
    xs = r_plan{1};
    ys = r_plan{2};
    r_traj = [xs', ys'];
    scatter(xs, ys);
end
xlim([robot_params.gmin(1), robot_params.gmax(1)]);
ylim([robot_params.gmin(2), robot_params.gmax(2)]);

%% Computes minimum distance between the hunan and robot trajectory.
function min_d = min_d_between_r_and_h(human_traj, robot_traj, car_rad)

    xs = robot_traj{1};
    ys = robot_traj{2};
    r_traj = [xs', ys'];
            
    h_traj = [human_traj{1}', human_traj{2}'];

    diff_hr = r_traj - h_traj;

    % do collision checking between the body of the robot and human 
    % vehicle which are both approximated by circles w/radius circle_rad
    d_to_h = sqrt(diff_hr(:,1).^2 + diff_hr(:,2).^2) - ...
                (car_rad + car_rad); 
            
    min_d = min(d_to_h);
end

%% Computes the distance from the end of the robot state to the goal
function d_to_goal = d_r_to_goal(robot_traj, goal)
    % only consider spatial dimension for now?
    final_state = [robot_traj{1}(end), robot_traj{2}(end)];
    d_to_goal = norm(final_state - goal(1:2));
end
clear all
close all

%% choose which dynamics model are the results for. 
% simple
% complex
model_type = 'simple';

if strcmp(model_type, 'simple')
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
    measurement_freq = 0.0891;
else
    load('max_tte_g2_uth0.35_complex_v46.mat');
    belief_updater = params.dyn_sys;
    gnums = params.gnums;
    measurement_freq = 0.1250; 
end

%load('exp_3_heuristic_baseline_trajs_brancht0.5.mat');
%load('exp_3_heuristic_baseline_trajs_brancht0.3265.mat');
load('NEWCOSTFUN_exp_3_heuristic_baseline_trajs_brancht0.3265.mat');

%% Save out important metrics.
all_d_to_goal_5050 = [];
all_d_to_human_5050 = [];

all_d_to_goal_correct = [];
all_d_to_human_correct = [];

all_d_to_goal_incorrect = [];
all_d_to_human_incorrect = [];

% Setup measurement frequency in (s)
robot_dt = robot_params.dt;
num_obs_per_dt = floor(robot_dt/measurement_freq);

sim_idx = 1;
for ri = 1:length(all_r_x0s)
    for hi = 1:length(all_h_x0s)
        for pgi = 1:length(all_priors)
            for gi=1:length(all_h_goals)
                goal = all_h_goals{gi};
                g1_preds = all_g1_preds{sim_idx};
                g2_preds = all_g2_preds{sim_idx};
                r_plan = all_plans{sim_idx};
                pgoals = all_priors{pgi};     
                
                if strcmp(goal, 'g1')
                    human_traj = g1_preds;
                    true_g = 1;
                else
                    human_traj = g2_preds;
                    true_g = 2;
                end
                
                if strcmp(model_type, 'simple')
                    % Compute what posterior would be after branch_t timesteps.
                    posterior = compute_posterior(human_traj, pgoals, ...
                                            robot_params.planner.times, branch_t, ...
                                            num_obs_per_dt, measurement_freq, ...
                                        gnums, belief_updater);
                else
                    % get the optimal policy from the MDP. 
                    curr_state = {human_traj{1}(1), human_traj{2}(1), human_traj{5}(1)};
                    [opt_mdp_traj, opt_mdp_ctrls] = belief_updater.get_opt_traj(curr_state, true_g);
                    % update posterior.
                    posterior = compute_posterior_mdp(human_traj, pgoals, ...
                                            robot_params.planner.times, ...
                                            branch_t, belief_updater, ...
                                            opt_mdp_traj, opt_mdp_ctrls);
                end
                [maxval, max_gidx] = max(posterior);
                %max_gidx = gi;
                                    
                % Compute dist between robot and goal. 
                d_to_goal = d_r_to_goal(r_plan, robot_params.goal, max_gidx);

                % Compute minimum distance between robot and human.
                min_d_to_human = min_d_between_r_and_h(human_traj, r_plan, ...
                                                        robot_params.car_rad, max_gidx);

                if max_gidx == true_g              
                    fprintf('Estimated true goal!\n');
                else
                    fprintf('Did *not* estimate true goal!\n');
                end
                
                if min_d_to_human < 0
                    fprintf('ri = %d, hi = %d, pgi = %d, gi = %d\n', ri, hi, pgi, gi);
                    fprintf('sim_idx = %d\n', sim_idx);
                end
                                                    
                % Store data.
                if pgoals(1) == 0.5 && pgoals(2) == 0.5
                    all_d_to_goal_5050 = [all_d_to_goal_5050, d_to_goal];
                    all_d_to_human_5050 = [all_d_to_human_5050, min_d_to_human];
                else
                    [~, likely_g] = max(pgoals);
                    %likely_g = max_gidx;
                    if strcmp(goal, 'g1')
                        true_g = 1;
                    else
                        true_g = 2;
                    end
                    
                    if likely_g == true_g
                        %fprintf('max_gidx: %d');
                        all_d_to_goal_correct = [all_d_to_goal_correct, d_to_goal];
                        all_d_to_human_correct = [all_d_to_human_correct, min_d_to_human];
                    else
                        all_d_to_goal_incorrect = [all_d_to_goal_incorrect, d_to_goal];
                        all_d_to_human_incorrect = [all_d_to_human_incorrect, min_d_to_human];
                    end
                end
            end
            
            sim_idx = sim_idx + 1;
        end
    end
end

% Avg of distance to goal.
%avg_d_to_g = mean(all_d_to_goal);
%std_d_to_g = std(all_d_to_goal);

% Avg of distance to human.
avg_d_to_h_5050 = mean(all_d_to_human_5050);
std_d_to_h_5050 = std(all_d_to_human_5050);
avg_d_to_goal_5050 = mean(all_d_to_goal_5050);
std_d_to_goal_5050 = std(all_d_to_goal_5050);

avg_d_to_h_correct = mean(all_d_to_human_correct);
std_d_to_h_correct = std(all_d_to_human_correct);
avg_d_to_goal_correct = mean(all_d_to_goal_correct);
std_d_to_goal_correct = std(all_d_to_goal_correct);

avg_d_to_h_incorrect = mean(all_d_to_human_incorrect);
std_d_to_h_incorrect = std(all_d_to_human_incorrect);
avg_d_to_goal_incorrect = mean(all_d_to_goal_incorrect);
std_d_to_goal_incorrect = std(all_d_to_goal_incorrect);

fprintf('============= HEURISTIC BASELINE: ==========\n');
fprintf('5050 Prior: \n');
fprintf('--> average distance to goal: %f (m)\n', avg_d_to_goal_5050);
fprintf('    stdev distance to goal: %f (m)\n', std_d_to_goal_5050);
fprintf('--> average (min) distance to human: %f (m)\n', avg_d_to_h_5050);
fprintf('    stdev (min) distance to human: %f (m)\n', std_d_to_h_5050);
fprintf('Correct Prior: \n');
fprintf('--> average distance to goal: %f (m)\n', avg_d_to_goal_correct);
fprintf('    stdev distance to goal: %f (m)\n', std_d_to_goal_correct);
fprintf('--> average (min) distance to human: %f (m)\n', avg_d_to_h_correct);
fprintf('    stdev (min) distance to human: %f (m)\n', std_d_to_h_correct);
fprintf('IN-Correct Prior: \n');
fprintf('--> average distance to goal: %f (m)\n', avg_d_to_goal_incorrect);
fprintf('    stdev distance to goal: %f (m)\n', std_d_to_goal_incorrect);
fprintf('--> average (min) distance to human: %f (m)\n', avg_d_to_h_incorrect);
fprintf('    stdev (min) distance to human: %f (m)\n', std_d_to_h_incorrect);
fprintf('=======================================\n');

%% SANITY CHECK!
%  plot the safe trajectories.
figure(1)
hold on
for i = 1:length(all_plans)
    r_plan = all_plans{i};
    shared = r_plan{1};
    branch_g1 = r_plan{2};
    branch_g2 = r_plan{3};
    xs_shared = shared{1};
    ys_shared = shared{2};
    xs_g1 = branch_g1{1};
    ys_g1 = branch_g1{2};
    xs_g2 = branch_g2{1};
    ys_g2 = branch_g2{2};
    c = [rand, rand, rand];
    scatter(xs_shared, ys_shared, 'markeredgecolor', c);
    scatter(xs_g1, ys_g1, 'markeredgecolor',c);
    scatter(xs_g2, ys_g2, 'markeredgecolor',c);
end
xlim([robot_params.gmin(1), robot_params.gmax(1)]);
ylim([robot_params.gmin(2), robot_params.gmax(2)]);

%% Computes minimum distance between the hunan and robot trajectory.
function min_d = min_d_between_r_and_h(human_traj, robot_traj, car_rad, max_gidx)

    shared_traj = robot_traj{1};
    
    if max_gidx == 1
        branch_traj = robot_traj{2};
    elseif max_gidx == 2
        branch_traj = robot_traj{3};
    else
        error('invalid max g idx!');
    end
    
    xs_shared = shared_traj{1};
    ys_shared = shared_traj{2};
    xs_branch = branch_traj{1}(2:end); % don't include shared point that is repeated. 
    ys_branch = branch_traj{2}(2:end);
    r_traj = [xs_shared', ys_shared'];
    r_traj = vertcat(r_traj, [xs_branch', ys_branch']);
            
    h_traj = [human_traj{1}', human_traj{2}'];

    diff_hr = r_traj - h_traj;

    % do collision checking between the body of the robot and human 
    % vehicle which are both approximated by circles w/radius circle_rad
    d_to_h = sqrt(diff_hr(:,1).^2 + diff_hr(:,2).^2) - ...
                (car_rad + car_rad); 
            
    min_d = min(d_to_h);
end

%% Computes the distance from the end of the robot state to the goal
function d_to_goal = d_r_to_goal(robot_traj, goal, max_gidx)

    if max_gidx == 1
        branch_traj = robot_traj{2};
    elseif max_gidx == 2
        branch_traj = robot_traj{3};
    else
        error('invalid max g idx!');
    end
    
    % only consider spatial dimension for now?
    final_state = [branch_traj{1}(end), branch_traj{2}(end)];
    d_to_goal = norm(final_state - goal(1:2));
end

%% Compute the belief after branch_t observations.
function posterior = compute_posterior_mdp(human_traj, prior, times, ...
                                        branch_t, belief_updater, ...
                                        opt_mdp_traj, opt_mdp_ctrls)
    % Compute number of waypoints in each part of the plan.
    [~,end_idx] = min(abs(times - branch_t));
    shared_num_waypts = end_idx;
    
    % Should we remove the last timestep because at the last timestep we 
    % need to commit to one of the two branched trajectories?
    shared_num_waypts = shared_num_waypts-1;
    
    total_human_traj_time = times(end);
    human_dt = times(2) - times(1);
    total_mdp_time = belief_updater.dt*length(opt_mdp_traj);
    mdp_dt = belief_updater.dt;
        
    % NOTE: THIS ASSUMES WE GET ONE OBSERVATION DURING EACH ROBOT TIMESTEP!
    posterior = prior;
    for t=2:shared_num_waypts
        
        curr_human_t = human_dt*(t);
        percent_total_t = curr_human_t/total_human_traj_time;
        
        curr_mdp_t = percent_total_t*total_mdp_time;
        curr_mdp_tidx = round(curr_mdp_t/mdp_dt);

        % Get the action the human took. 
        h_xcurr = opt_mdp_traj{curr_mdp_tidx};
        uidx = opt_mdp_ctrls{curr_mdp_tidx};
        
        % Create joint state!
        z = {h_xcurr{1}, h_xcurr{2}, h_xcurr{3}, posterior(1)};
        
        % Update posterior!
        posterior_g1 = belief_updater.belief_update(uidx,z);
        posterior = [posterior_g1, 1-posterior_g1];
    end
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
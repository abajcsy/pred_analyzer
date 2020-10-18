classdef ConfContingencyPlanner < handle
    %CONTINGENCYPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        num_waypts
        horizon
        goal
        max_linear_vel
        max_angular_vel
        footprint_rad
        sd_obs
        sd_goal
        g2d
        g3d
        disc_xy
        disc_xyth
        disc_xythvel
        dyn_feas_xythvel
        feasible_final_goals
        gmin
        gmax
        gnums
        opt_preds
        frs_preds
        pred_times
        times
        dt
        belief
        pthresh
        pred_g
    end
    
    methods
        %% Constructs Contingency Planner. 
        function obj = ConfContingencyPlanner(num_waypts, horizon, ...
                                        goal, ...
                                        max_linear_vel, ...
                                        max_angular_vel, ...
                                        footprint_rad, ...
                                        sd_obs, ...
                                        sd_goal, ...
                                        g2d, ...
                                        g3d, ...
                                        belief, ...
                                        gmin, gmax, gnums, ...
                                        pthresh, ...
                                        dv)
            obj.num_waypts = num_waypts;
            obj.horizon = horizon;
            obj.goal = goal;
            obj.max_linear_vel = max_linear_vel;
            obj.max_angular_vel = max_angular_vel;
            obj.footprint_rad = footprint_rad;
            obj.sd_obs = sd_obs;
            obj.sd_goal = sd_goal;
            obj.g2d = g2d;
            obj.g3d = g3d;
            obj.gmin = gmin;
            obj.gmax = gmax;
            obj.gnums = gnums;
            obj.pthresh = pthresh;
            obj.belief = belief;
            
            % Discretization of x, y, theta, and velocity space
            gdisc = (gmax - gmin) ./ (gnums - 1);
            [X2D,Y2D] = meshgrid(gmin(1):gdisc(1):gmax(1), ...
                             gmin(2):gdisc(2):gmax(2));
            [X3D,Y3D,TH3D] = meshgrid(obj.g3d.min(1):obj.g3d.dx(1):obj.g3d.max(1), ...
                             obj.g3d.min(2):obj.g3d.dx(2):obj.g3d.max(2), ...
                             obj.g3d.min(3):obj.g3d.dx(3):obj.g3d.max(3));
            [X,Y,TH,V] = ndgrid(obj.g3d.min(1):obj.g3d.dx(1):obj.g3d.max(1), ...
                             obj.g3d.min(2):obj.g3d.dx(2):obj.g3d.max(2), ...
                             obj.g3d.min(3):obj.g3d.dx(3):obj.g3d.max(3), ... %linspace(-3*pi/4, 3*pi/4, obj.g3d.N(3)), ...
                             0.01:dv:obj.max_linear_vel); 
                             %obj.g3d.min(3):obj.g3d.dx(3):obj.g3d.max(3), ...
                             %-obj.max_linear_vel:dv:obj.max_linear_vel);             
            obj.disc_xy = [X2D(:), Y2D(:)];
            obj.disc_xyth = [X3D(:), Y3D(:), TH3D(:)];
            obj.disc_xythvel = [X(:), Y(:), TH(:), V(:)];
        end
        
        %% Helper function which pre-computes all shared goals
        %  which are dynamically feasible within the branching horizon. 
        %  (this is used to speed up the contingency planner parameter search) 
        function dyn_feas_xythvel = ...
                find_dyn_feas_shared_goal(obj, start, horiz, num_waypts)
            dyn_feas_xythvel = [];
            for i=1:length(obj.disc_xythvel)
                candidate_goal = obj.disc_xythvel(i,:);
                
                % ignore candidate goals inside obstacles.
                if eval_u(obj.g2d, obj.sd_obs, candidate_goal(1:2)) < 0
                    continue;
                end
                
                curr_spline = ...
                    spline(start, candidate_goal, horiz, num_waypts);
                feasible_horiz = obj.compute_dyn_feasible_horizon(curr_spline, ...
                                                  obj.max_linear_vel, ...
                                                  obj.max_angular_vel, ...
                                                  horiz);
                
                if feasible_horiz <= horiz
                    dyn_feas_xythvel = [dyn_feas_xythvel; candidate_goal];
                    
%                     figure(3)
%                     hold on
%                     scatter(candidate_goal(1), candidate_goal(2), 'g');
%                     xlim([-6.5,6.5]);
%                     ylim([-6.5,6.5]);
%                     grid on
                end
            end
        end
        
        %% Plans a contingency plan from start to goal.
        function opt_plan = ...
                contingency_plan(obj, start, goal, ...
                                opt_preds, frs_preds, pred_times, ...
                                pred_g, belief, branch_t)
            % Update belief
            obj.belief = belief;
            
            % Grab all of the predictions and corresponding times.
            % Assumes that 
            obj.opt_preds = opt_preds;
            obj.frs_preds = frs_preds;
            obj.pred_times = pred_times;
            obj.pred_g = pred_g;
            
            % Times along spline. 
            obj.times = linspace(0, obj.horizon, obj.num_waypts); %pred_times; 
            obj.dt = obj.times(2) - obj.times(1); 
            
            % Compute number of waypoints in each part of the plan.
            [~,end_idx] = min(abs(obj.times - branch_t));
            
            %% OVERESTIMATE THE TIME TO BRANCH
            end_idx = end_idx+1;
            binned_branch_t = obj.times(end_idx); 
            shared_num_waypts = end_idx;
            branch_num_waypts = abs(obj.num_waypts - end_idx)+1;
            
            binned_horiz = obj.times(end);
            
            % Keep track of best spline.
            opt_reward = -100000000000000.0;
            opt_plan = {};
            
            % Precompute the set of dynamically feasible goal states 
            % at which we branch into contingency plans. 
            % Used to save on compute time. 
            feasible_shared_goals = ...
                obj.find_dyn_feas_shared_goal(start, ...
                                              binned_branch_t, ...
                                              shared_num_waypts);
            
            % Three options for how to search for shared goal:
            % Option 1:
            [num_shared_goals,~] = size(feasible_shared_goals); 
            % Option 2:
            %num_shared_goals = length(obj.disc_xythvel); 
            % Option 3:
            % % set random seed
            % % rng(5);
            %num_shared_goals = 500; % get a 500 samples 
            
            for i=1:num_shared_goals 
                fprintf('Evaluating %d / %d (%f percent)...\n', ...
                    i, num_shared_goals, 100*(i /num_shared_goals));
                
                % Option 1: search only through precomputed dyn feasible
                % goals.
                shared_goal = feasible_shared_goals(i, :);
                
                % Option 2: full grid search over parameters.
                %shared_goal = obj.disc_xythvel(i, :);
                
                % Option 3: randomly sample a shared goal. 
                %rand_idx = randi(length(obj.disc_xythvel));
                %shared_goal = obj.disc_xythvel(rand_idx, :);
                % % resample if candidate goal is inside obstacles.
                %while eval_u(obj.g2d, obj.sd_obs, shared_goal(1:2)) < 0
                %    rand_idx = randi(length(obj.disc_xythvel));
                %    shared_goal = obj.disc_xythvel(rand_idx, :);
                %end
                 
                % Compute shared spline. 
                shared_spline = spline(start, shared_goal, ...
                                        binned_branch_t, shared_num_waypts);

                % Extract final state of shared spline.
                start_branch = [shared_spline{1}(end), ...
                                shared_spline{2}(end), ...
                                shared_spline{5}(end), ...
                                shared_spline{3}(end)]; % (x,y,th,lin_v)        

                % Compute branching splines for g1 and g2   
                spline_opt = obj.plan(start_branch, goal, binned_branch_t, ...
                                        binned_horiz, branch_num_waypts, 'opt');
                spline_frs = obj.plan(start_branch, goal, binned_branch_t, ...
                                        binned_horiz, branch_num_waypts, 'frs');
                                    
                if isempty(spline_opt) || isempty(spline_frs)
                    continue;
                end
                
                % Sanity check (and correct) all points on spline to be within env bounds. 
                total_spline1 = {cat(2,shared_spline{1},spline_opt{1}(2:end)), ...
                                 cat(2,shared_spline{2},spline_opt{2}(2:end)), ...
                                 cat(2,shared_spline{3},spline_opt{3}(2:end)), ...
                                 cat(2,shared_spline{4},spline_opt{4}(2:end)), ...
                                 cat(2,shared_spline{5},spline_opt{5}(2:end))};
                total_spline2 = {cat(2,shared_spline{1},spline_frs{1}(2:end)), ...
                                 cat(2,shared_spline{2},spline_frs{2}(2:end)), ...
                                 cat(2,shared_spline{3},spline_frs{3}(2:end)), ...
                                 cat(2,shared_spline{4},spline_frs{4}(2:end)), ...
                                 cat(2,shared_spline{5},spline_frs{5}(2:end))};   
                             
                % Sanity check (and correct) all points on spline to be within env bounds.              
                total_spline1 = obj.sanity_check_spline(total_spline1, obj.num_waypts);
                total_spline2 = obj.sanity_check_spline(total_spline2, obj.num_waypts);
                
                % Compute the dynamically feasible horizon for the current plan.
                feasible_horizon1 = ...
                    obj.compute_dyn_feasible_horizon(total_spline1, ...
                                                  obj.max_linear_vel, ...
                                                  obj.max_angular_vel, ...
                                                  binned_horiz);
                feasible_horizon2 = ...
                    obj.compute_dyn_feasible_horizon(total_spline2, ...
                                                  obj.max_linear_vel, ...
                                                  obj.max_angular_vel, ...
                                                  binned_horiz); 
%                 figure(2)   
%                 clf                              
%                 hold on
%                 qs = quiver(shared_spline{1}, shared_spline{2}, cos(shared_spline{5}), sin(shared_spline{5}), 'm');
%                 qs.AutoScale = 'off';
%                 qs.AutoScaleFactor = 0.5;
%                 qs = quiver(spline_opt{1}, spline_opt{2}, cos(spline_opt{5}), sin(spline_opt{5}), 'r');
%                 qs.AutoScale = 'off';
%                 qs.AutoScaleFactor = 0.5;
%                 qs = quiver(spline_frs{1}, spline_frs{2}, cos(spline_frs{5}), sin(spline_frs{5}), 'b');
%                 qs.AutoScale = 'off';
%                 qs.AutoScaleFactor = 0.5;
%                 %plot(spline_opt{1}, spline_opt{2}, 'r');
%                 %plot(spline_frs{1}, spline_frs{2}, 'b');
%                 xlim([-6.5,6.5]);
%                 ylim([-6.5,6.5]);
%                 hold off
%                 grid on

                % If total plan for each contingency plan is dynamically
                % feasible, then evaluate reward. 
                if (feasible_horizon1 <= obj.horizon && ...
                        feasible_horizon2 <= obj.horizon)
                    
                    % Get reward of each trajectory segment
                    reward_shared = obj.eval_reward(shared_spline, 'all', 0, binned_branch_t);
                    reward_opt = obj.eval_reward(spline_opt, 'opt', binned_branch_t, binned_horiz);
                    reward_frs = obj.eval_reward(spline_frs, 'frs', binned_branch_t, binned_horiz);
                    
                    % Compute total reward where we weight the reward
                    % contribution of contingency plans based on the
                    % likelihood of that goal occuring. 
                    %
                    %   R(traj) = R(shared) + \sum^M_i=1 b(g_i) * R(contingency_i)
                    %
                    % obj.belief(1) is for low-confidence model (i.e. FRS)
                    % obj.belief(2) is for high-confidence model (i.e. opt)
                    reward = reward_shared + ...
                             obj.belief(1) * reward_frs + ...
                             obj.belief(2) * reward_opt;
                        
                    if (reward > opt_reward)
%                         
%                         figure(3)
%                         hold on
%                         plot(shared_spline{1}, shared_spline{2}, 'm');
%                         plot(spline_opt{1}, spline_opt{2}, 'r');
%                         plot(spline_frs{1}, spline_frs{2}, 'b');
%                         xlim([-6.5,6.5]);
%                         ylim([-6.5,6.5]);
%                         grid on
                        
                        opt_reward = reward;
                        opt_plan = {shared_spline, spline_opt, spline_frs};                        
                    end
                end
            end
            
            if isempty(opt_plan)
                error("Unable to find dynamically feasible and low cost contingency plan!");
            end
            
        end
        
        %% Plans a single path from start to goal. 
        %   start   (arr) -- contains x,y,theta,vel initial state
        %   goal    (arr) -- contains x,y,theta,vel goal state
        %   hor     (float) -- time horizon (in seconds)
        %   coll_check (string) -- which predictions to collision check
        %                           against. options: 'all', 'opt', 'frs') 
        function opt_spline = plan(obj, start, goal, ...
                                    start_t, hor, num_waypts, coll_check)
            if nargin < 7
                coll_check = 'all';
            end
            
            opt_reward = -100000000000000.0;
            opt_spline = {};
            curr_spline = {};
            
            % DEBUGGING
            %figure
            all_rewards = [];
            plt_handles = {};
            
            %num_candidate_goals = length(obj.feasible_final_goals); 
            num_candidate_goals = length(obj.disc_xy); 
            %num_candidate_goals = 100;
            
            for ti=1:num_candidate_goals
                candidate_goal = obj.disc_xy(ti, :); 
                %candidate_goal = obj.feasible_final_goals(ti, :);
                
                %rand_idx = randi(num_candidate_goals);
                %candidate_goal = obj.disc_xy(rand_idx, :); 
                
                % ignore candidate goals inside obstacles.
                if eval_u(obj.g2d, obj.sd_obs, candidate_goal(1:2)) < 0
                    continue;
                end
                
                % orientation should match with goal final vel ~= 0.
                candidate_goal = [candidate_goal, goal(3), 0.01]; 
                
                % Compute spline from start to candidate (x,y) goal. 
                % curr_spline = {xs, ys, u1_lin_vel, u2_ang_vel}
                curr_spline = ...
                    spline(start, candidate_goal, hor-start_t, num_waypts);
                
                % Sanity check (and correct) all points on spline to be within env bounds. 
                curr_spline = obj.sanity_check_spline(curr_spline, num_waypts);
                
                % Compute the dynamically feasible horizon for the current plan.
                feasible_horizon = ...
                    obj.compute_dyn_feasible_horizon(curr_spline, ...
                      obj.max_linear_vel, ...
                      obj.max_angular_vel, ...
                      hor-start_t);
                  
                % If current spline is dyamically feasible, check if it is low cost.
                if (feasible_horizon <= hor-start_t)
                    
%                     if strcmp(coll_check, 'g2')
%                         hold on
%                         contour(obj.g2d.xs{1}, obj.g2d.xs{2}, obj.sd_obs, [0,0]);
%                         colors = [linspace(0,1,length(curr_spline{1}))', ...
%                         zeros([length(curr_spline{1}), 1]), ...
%                         zeros([length(curr_spline{1}), 1])];
%                         p = plot(curr_spline{1}, curr_spline{2});
%                         xlim([obj.gmin(1),obj.gmax(1)]);
%                         ylim([obj.gmin(2),obj.gmax(2)]);
%                         grid on
%                     end
%                         
%                     if strcmp(coll_check, 'g2') && (ti == 280 || ti == 1918) %(ti == 240 || ti == 241)
%                         bla = 1;
%                     end
                    
                    reward = obj.eval_reward(curr_spline, coll_check, start_t, hor);
                        
                    if (reward > opt_reward)
                        opt_reward = reward;
                        opt_spline = curr_spline;
                    end
                end
            end
            
%             c = linspace(0.1,0.9,length(plt_handles));
%             for i=1:length(plt_handles)
%                 plt_handles{i}.Color = [c(i),0,0];
%                 plt_handles{i}.LineWidth = 2;
%             end
%             p = plot(opt_spline{1}, opt_spline{2}, 'Color', 'b', 'LineWidth', 2);
%             
%             if isempty(opt_spline)
%                 error("Unable to find dynamically feasible and low cost spline plan!");
%             end
            
        end
        
        %% Check (and correct) if spline is inside environment bounds.
        function checked_spline = sanity_check_spline(obj, curr_spline, num_waypts)
            checked_spline = curr_spline;
            
            % TODO: can optimize this by MATLAB-ifying it. 
            for i=1:num_waypts
                x = checked_spline{1}(i);
                y = checked_spline{2}(i);

                if (x > obj.gmax(1)) 
                  checked_spline{1}(i) = obj.gmax(1);
                end
                if (y > obj.gmax(2)) 
                  checked_spline{2}(i) = obj.gmax(2);
                end
                if (x < obj.gmin(1)) 
                  checked_spline{1}(i) = obj.gmin(1);
                end
                if (y < obj.gmin(2)) 
                  checked_spline{2}(i) = obj.gmin(2);
                end
            end
        end
        
        %% Computes dynamically feasible horizon (given dynamics of car).
        function feasible_horizon = ...
                compute_dyn_feasible_horizon(obj, spline, ...
                                              max_linear_vel, ...
                                              max_angular_vel, ...
                                              final_t)
              % Compute max linear and angular speed.
              plan_max_lin_vel = max(spline{3});
              plan_max_angular_vel = max(abs(spline{4}));
              
              % Compute required horizon to acheive max speed of planned spline.
              feasible_horizon_speed = ...
                  final_t * plan_max_lin_vel / max_linear_vel;

              % Compute required horizon to acheive max angular vel of planned spline.
              feasible_horizon_angular_vel = ...
                  final_t * plan_max_angular_vel / max_angular_vel;

              feasible_horizon = ...
                  max(feasible_horizon_speed, feasible_horizon_angular_vel);
        end
        
        %% Evaluates the total reward along the trajectory. 
        function reward = eval_reward(obj, curr_spline, coll_check, start_t, hor)            
            xs = curr_spline{1};
            ys = curr_spline{2};
            ths = curr_spline{5};
            traj = [xs', ys'];
                       
            % TODO: add in penalty for orientation too?
            % TODO: add in penalty for human-driven vehicle prediction. 
            obs_r = eval_u(obj.g2d, obj.sd_obs, traj);
            goal_r = eval_u(obj.g2d, obj.sd_goal, traj);
            
            % for each time, compute if the robot state is in the human
            % state.
            
            % Find the time index to start and end the collision-checking 
            % with the human trajectories.
            [~,plan_start_idx] = min(abs(obj.times - start_t));
            [~,plan_end_idx] = min(abs(obj.times - hor));
            
            [~,pred_start_idx] = min(abs(obj.pred_times - start_t));
            [~,pred_end_idx] = min(abs(obj.pred_times - hor));
            
            relevent_plan_times = obj.times(plan_start_idx:plan_end_idx);
            relevent_pred_times = obj.pred_times(pred_start_idx:pred_end_idx);
            frs_relevant_preds = obj.frs_preds(1,pred_start_idx:pred_end_idx);
            opt_relevant_preds = obj.opt_preds(1,pred_start_idx:pred_end_idx);
            
            human_r = zeros(size(obs_r));
            for i=1:length(traj)
                curr_t = relevent_plan_times(i);
                % find upper and lower times in preds for the current time.
                [~, idx] = min(abs(relevent_pred_times - curr_t));
                lower_idx = idx; 
                upper_idx = idx;
                if curr_t < relevent_pred_times(idx)
                    lower_t = relevent_pred_times(idx);
                    upper_t = relevent_pred_times(idx);
                    alpha = 1.0;
                elseif curr_t > relevent_pred_times(end) 
                    lower_t = relevent_pred_times(idx);
                    upper_t = relevent_pred_times(idx);
                    alpha = 0.0;
                else
                    lower_t = relevent_pred_times(idx);
                    upper_t = relevent_pred_times(idx+1);
                    lower_idx = idx; 
                    upper_idx = idx+1;
                    alpha = (curr_t - lower_t) / (upper_t - lower_t);
                end
                
                r = 0.0;
                % compute the human reward.
                if strcmp(coll_check, 'all') || strcmp(coll_check, 'frs')
                    lower_pred = frs_relevant_preds{lower_idx};
                    upper_pred = frs_relevant_preds{upper_idx};
                    final_pred = alpha .* lower_pred + (1-alpha) .* upper_pred;
                    val = eval_u(obj.pred_g, final_pred, traj(i,:));
                    if val > 0.0 %obj.pthresh
                        r = -150.0;
                    end
                elseif strcmp(coll_check, 'opt')
                    lower_pred = opt_relevant_preds{lower_idx};
                    upper_pred = opt_relevant_preds{upper_idx};
                    final_pred = alpha .* lower_pred + (1-alpha) .* upper_pred;
                    val = eval_u(obj.pred_g, final_pred, traj(i,:));
                    if val > 0.0 %obj.pthresh
                        r = -150.0;
                    end
                else
                    error('Invalid collision check option! %s', coll_check);
                end
                human_r(i) = r;
            end
            
            reward = sum(obs_r + goal_r + human_r); 
        end
        
        %% Grab all the likely-enough predicted states.
        function opt_eps = compute_likely_states(obj, preds, eps)

            valid_indices = find(preds > 0);
            valid_data = preds(valid_indices);
            sorted_valid_data = sort(valid_data, 'descend');
            eps_index = find(cumsum(sorted_valid_data) > (1 - eps), 1, 'first');

            if isempty(eps_index)
                % if we can't find likely enough states, then we should take max. 
                eps_index = find(max(cumsum(sorted_valid_data)), 1, 'first');
            end

            opt_eps = sorted_valid_data(eps_index);
        end
        
        %% Updates the signed distance to goal.
        function obj = set_sd_goal(obj, sd_goal_new)
            obj.sd_goal = sd_goal_new;
        end
        
        %% Returns the four corners of the rotated rectangle with center
        %  at (xc, yc) and angle of rotation th with side lens + widths.
        function [ur, ul, dl, dr] = ....
            get_four_corners_rot_rect(obj, xc, yc, th, car_len, car_width)

            rot_mat = [cos(th) -sin(th); ... 
                       sin(th) cos(th)];

            xoff = 0.5*car_len;
            yoff = 0.5*car_width;

            ur = [xc; yc] + rot_mat * [xoff; yoff];
            ul = [xc; yc] + rot_mat * [- xoff; yoff]; 
            dl = [xc; yc] + rot_mat * [- xoff; - yoff]; 
            dr = [xc; yc] + rot_mat * [xoff; - yoff]; 
        end
        
    end
end


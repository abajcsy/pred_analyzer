classdef ConfSplinePlanner < handle
    %CONFSPLINEPLANNER Summary of this class goes here
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
        gmin
        gmax
        gnums
        %opt_preds
        %frs_preds
        %conf_preds
        pred_times
        pthresh
        pred_g
        times
        dt
    end
    
    methods
        %% Constructs Confidence-aware Spline Planner. 
        function obj = ConfSplinePlanner(num_waypts, horizon, ...
                                        goal, ...
                                        max_linear_vel, ...
                                        max_angular_vel, ...
                                        footprint_rad, ...
                                        sd_obs, ...
                                        sd_goal, ...
                                        g2d, ...
                                        g3d, ...
                                        gmin, gmax, gnums, ...
                                        pthresh)
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
            
            % Grab all of the predictions and corresponding times.
            % Assumes that 
%             obj.opt_preds = opt_preds;
%             obj.frs_preds = frs_preds;
%             obj.conf_preds = conf_preds;
%             obj.pred_times = pred_times;
%             obj.pred_g = pred_g;
            
            % Times along spline. 
%             obj.times = pred_times; 
%             obj.dt = obj.times(2) - obj.times(1); 
            
            gdisc = (gmax - gmin) ./ (gnums - 1);
            [X2D,Y2D] = meshgrid(gmin(1):gdisc(1):gmax(1), ...
                             gmin(2):gdisc(2):gmax(2));
            [X,Y,TH] = meshgrid(obj.g3d.min(1):obj.g3d.dx(1):obj.g3d.max(1), ...
                 obj.g3d.min(2):obj.g3d.dx(2):obj.g3d.max(2), ...
                 obj.g3d.min(3):obj.g3d.dx(3):obj.g3d.max(3));
            obj.disc_xy = [X2D(:), Y2D(:)];
            obj.disc_xyth = [X(:), Y(:), TH(:)];
        end
        
        %% Plans a path from start to goal. 
        function opt_spline = plan(obj, start, goal, ...
                human_preds, pred_times, pred_g, coll_check)
            
            % Grab all of the predictions and corresponding times.
            %obj.times = pred_times; 
            %obj.dt = obj.times(2) - obj.times(1); 
            obj.pred_times = pred_times;
            obj.pred_g = pred_g;
            
            % Times along spline. 
            obj.times = linspace(0, obj.horizon, obj.num_waypts); 
            obj.dt = obj.times(2) - obj.times(1); 
            
            % Prediction grid. 
            obj.pred_g = pred_g;
            
            opt_reward = -100000000000000.0;
            opt_spline = {};
            curr_spline = {};
            
            % DEBUGGING
            %figure
            all_rewards = [];
            plt_handles = {};
            
            for ti=1:length(obj.disc_xy) %length(obj.disc_xyth)
                candidate_goal = obj.disc_xy(ti, :);%obj.disc_xyth(ti, :);
                
                % ignore candidate goals inside obstacles.
                if eval_u(obj.g2d, obj.sd_obs, candidate_goal(1:2)) < 0
                    continue;
                end
                
                % orientation should match with goal final vel ~= 0.
                candidate_goal = [candidate_goal, goal(3), 0.01]; %[candidate_goal, 0.01]; 
                
                % Compute spline from start to candidate (x,y) goal. 
                curr_spline = ...
                    spline(start, candidate_goal, obj.horizon, obj.num_waypts);
                
                % Sanity check (and correct) all points on spline to be within env bounds. 
                curr_spline = obj.sanity_check_spline(curr_spline);
                
                % Compute the dynamically feasible horizon for the current plan.
                feasible_horizon = ...
                    obj.compute_dyn_feasible_horizon(curr_spline, ...
                      obj.max_linear_vel, ...
                      obj.max_angular_vel, ...
                      obj.horizon);
                  
                % If current spline is dyamically feasible, check if it is low cost.
                if (feasible_horizon <= obj.horizon) 
                    reward = obj.eval_reward(curr_spline, human_preds, coll_check);

                    if (reward > opt_reward)
%                         figure(2)
%                         hold on
%                         plot(curr_spline{1}, curr_spline{2})
%                         xlim([-6,6])
%                         ylim([-6,6])
%                         hold off
                        
                        opt_reward = reward;
                        opt_spline = curr_spline;
                    end
                end
            end
            
            if isempty(opt_spline)
                error("Unable to find dynamically feasible and low cost spline plan!");
            end
            
        end
        
        %% Check (and correct) if spline is inside environment bounds.
        function checked_spline = sanity_check_spline(obj, curr_spline)
            checked_spline = curr_spline;
            
            % TODO: can optimize this by MATLAB-ifying it. 
            for i=1:obj.num_waypts 
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
        function reward = eval_reward(obj, curr_spline, human_preds, coll_check)            
            xs = curr_spline{1};
            ys = curr_spline{2};
            ths = curr_spline{5};
            traj = [xs', ys'];
            
            % TODO: add in penalty for orientation too?
            % TODO: add in penalty for human-driven vehicle prediction. 
            obs_r = eval_u(obj.g2d, obj.sd_obs, traj);
            goal_r = eval_u(obj.g2d, obj.sd_goal, traj);
            
            if any(obs_r) % if any obs_r is non-zero, 
                obs_r = ones(size(obs_r))*-10000.0;
            end
            
            % for each time, compute if the robot state is in the human
            % state.
            
            % Find the time index to start and end the collision-checking 
            % with the human trajectories.
            start_t = 0.0;
            hor = obj.horizon;
            [~,plan_start_idx] = min(abs(obj.times - start_t));
            [~,plan_end_idx] = min(abs(obj.times - hor));
            
            [~,pred_start_idx] = min(abs(obj.pred_times - start_t));
            [~,pred_end_idx] = min(abs(obj.pred_times - hor));
            
            relevent_plan_times = obj.times(plan_start_idx:plan_end_idx);
            relevent_pred_times = obj.pred_times(pred_start_idx:pred_end_idx);
            relevant_preds = human_preds(1,pred_start_idx:pred_end_idx);
            
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
                
                lower_pred = relevant_preds{lower_idx};
                upper_pred = relevant_preds{upper_idx};
                final_pred = alpha .* lower_pred + (1-alpha) .* upper_pred;
                
                if strcmp(coll_check, 'conf')
                    %% NOTE THIS IS A HACCCKKKK
                    opt_eps = obj.compute_likely_states(final_pred, obj.pthresh);
                else
                    opt_eps = 0.0;
                end
                
                % compute the human reward.
                val = eval_u(obj.pred_g, final_pred, traj(i,:));
                if val > opt_eps
                    r = -100.0;
                end
                
                human_r(i) = r;
            end
            
            reward = sum(obs_r + goal_r + human_r); 
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

        %% Plots human predictions.
        function plot_human_preds(obj, human_preds, pred_type)
            for t=1:length(human_preds)
                opt_eps = obj.compute_likely_states(human_preds{t}, obj.pthresh);
                thresholded_preds = (human_preds{t} >= opt_eps) .* 1.0 + (human_preds{t} < opt_eps) .* 0.0;
                pcolor(obj.pred_g.xs{1}, obj.pred_g.xs{2}, thresholded_preds);
                title(strcat('t=',num2str(obj.pred_times(t))));
                pause(0.1);
            end
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
        
    end
end


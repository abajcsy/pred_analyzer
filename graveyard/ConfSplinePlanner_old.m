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
        opt_preds
        frs_preds
        conf_preds
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
                                        opt_preds, ...
                                        frs_preds, ...
                                        conf_preds, ...
                                        pred_times, ...
                                        g2d, ...
                                        g3d, ...
                                        gmin, gmax, gnums, ...
                                        pthresh, ...
                                        pred_g)
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
            obj.opt_preds = opt_preds;
            obj.frs_preds = frs_preds;
            obj.conf_preds = conf_preds;
            obj.pred_times = pred_times;
            obj.pred_g = pred_g;
            
            % Times along spline. 
            obj.times = pred_times; %linspace(0,obj.horizon,obj.num_waypts);
            obj.dt = obj.times(2) - obj.times(1); 
            
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
        function opt_spline = plan(obj, start, goal, coll_check)
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
                    reward = obj.eval_reward(curr_spline, coll_check);

                    if (reward > opt_reward)
                        opt_reward = reward;
                        opt_spline = curr_spline;
                        
                                            
%                         % blaaa
%                         hold on
%                         contour(obj.g2d.xs{1}, obj.g2d.xs{2}, obj.sd_obs, [0,0]);
%                         colors = [linspace(0,1,length(curr_spline{1}))', ...
%                                     zeros([length(curr_spline{1}), 1]), ...
%                                     zeros([length(curr_spline{1}), 1])];
%                         p = plot(curr_spline{1}, curr_spline{2});
%                         xlim([obj.gmin(1),obj.gmax(1)]);
%                         ylim([obj.gmin(2),obj.gmax(2)]);
%                         all_rewards(end+1) = reward;
%                         plt_handles{end+1} = p;
%                         grid on
%                         % blaaa
                    
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
              plan_max_angular_vel = max(spline{4});
              
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
        function reward = eval_reward(obj, curr_spline, coll_check)            
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
            % NOTE: ASSUMES THAT PREDICTION LENGTH AND TIME IS SAME AS
            % SPLINE PLAN LENGTH AND TIME.
            % circle_rad = 1.2;
            
            human_r = zeros(size(obs_r));
            for i=1:length(traj)
                r = 0.0;
                % compute the human reward.
                if strcmp(coll_check, 'all') || strcmp(coll_check, 'frs')
                    val = eval_u(obj.pred_g, obj.frs_preds{i}, traj(i,:));
                    if val > obj.pthresh
                        r = -100.0;
                    end
                elseif strcmp(coll_check, 'opt')
                    val = eval_u(obj.pred_g, obj.opt_preds{i}, traj(i,:));
                    if val > obj.pthresh
                        r = -100.0;
                    end
                elseif strcmp(coll_check, 'conf')
                    opt_eps = obj.compute_likely_states(obj.conf_preds{i}, obj.pthresh);
                    val = eval_u(obj.pred_g, obj.conf_preds{i}, traj(i,:));
                    if val > opt_eps
                        r = -100.0;
                    end
                else
                    error('Invalid collision check option! %s', coll_check);
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
        function plot_human_preds(obj, pred_type)
            if strcmp(pred_type, 'all') || strcmp(pred_type, 'frs')
                preds = obj.frs_preds;
            elseif strcmp(pred_type, 'opt')
                preds = obj.opt_preds;
            else % conf-aware preds
                preds = obj.conf_preds;
            end
            
            for t=1:length(preds)
                opt_eps = obj.compute_likely_states(preds{t}, obj.pthresh);
                thresholded_preds = (preds{t} >= opt_eps) .* 1.0 + (preds{t} < opt_eps) .* 0.0;
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


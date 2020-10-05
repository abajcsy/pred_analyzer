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
                                        opt_preds, ...
                                        frs_preds, ...
                                        pred_times, ...
                                        g2d, ...
                                        g3d, ...
                                        belief, ...
                                        gmin, gmax, gnums, ...
                                        pthresh, ...
                                        pred_g, ...
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
            
            % Grab all of the predictions and corresponding times.
            % Assumes that 
            obj.opt_preds = opt_preds;
            obj.frs_preds = frs_preds;
            obj.pred_times = pred_times;
            obj.pred_g = pred_g;
            
            % Times along spline. 
            obj.times = pred_times; %linspace(0,obj.horizon,obj.num_waypts);
            obj.dt = obj.times(2) - obj.times(1); 
            
            % Discretization of x, y, theta, and velocity space
            gdisc = (gmax - gmin) ./ (gnums - 1);
            [X2D,Y2D] = meshgrid(gmin(1):gdisc(1):gmax(1), ...
                             gmin(2):gdisc(2):gmax(2));
            [X3D,Y3D,TH3D] = meshgrid(obj.g3d.min(1):obj.g3d.dx(1):obj.g3d.max(1), ...
                             obj.g3d.min(2):obj.g3d.dx(2):obj.g3d.max(2), ...
                             obj.g3d.min(3):obj.g3d.dx(3):obj.g3d.max(3));
            [X,Y,TH,V] = ndgrid(obj.g3d.min(1):obj.g3d.dx(1):obj.g3d.max(1), ...
                             obj.g3d.min(2):obj.g3d.dx(2):obj.g3d.max(2), ...
                             obj.g3d.min(3):obj.g3d.dx(3):obj.g3d.max(3), ...
                             -obj.max_linear_vel:dv:obj.max_linear_vel);             
            obj.disc_xy = [X2D(:), Y2D(:)];
            obj.disc_xyth = [X3D(:), Y3D(:), TH3D(:)];
            obj.disc_xythvel = [X(:), Y(:), TH(:), V(:)];
        end
        
        %% Plans a contingency plan from start to goal.
        function opt_plan = ...
                contingency_plan(obj, start, goal, branch_t)
            
            % Compute number of waypoints in each part of the plan.
            [~,end_idx] = min(abs(obj.times - branch_t));
            shared_num_waypts = end_idx;
            branch_num_waypts = abs(obj.num_waypts - end_idx);
            
            % Keep track of best spline.
            opt_reward = -100000000000000.0;
            opt_plan = {};
            
            for i=1:length(obj.disc_xythvel)
                fprintf('Evaluating %d / %d (%f percent)...\n', ...
                    i, length(obj.disc_xythvel), 100*(i /length(obj.disc_xythvel)));
                
                shared_goal = obj.disc_xythvel(i, :);
                 
                % Compute shared spline. 
                shared_spline = spline(start, shared_goal, ...
                                        branch_t, shared_num_waypts);

                % Extract final state of shared spline.
                start_branch = [shared_spline{1}(end), ...
                                shared_spline{2}(end), ...
                                shared_spline{5}(end), ...
                                shared_spline{3}(end)]; % (x,y,th,lin_v)        

                % Compute branching splines for g1 and g2   
%                 spline_opt = spline(start_branch, goal, ...
%                                     obj.horizon-(branch_t+obj.dt), branch_num_waypts);
%                 spline_frs = spline(start_branch, goal, ...
%                                     obj.horizon-(branch_t+obj.dt), branch_num_waypts);

                spline_opt = obj.plan(start_branch, goal, branch_t, ...
                                        obj.horizon-obj.dt, branch_num_waypts, 'opt');
                spline_frs = obj.plan(start_branch, goal, branch_t, ...
                                        obj.horizon-obj.dt, branch_num_waypts, 'frs');
                                    
                if isempty(spline_opt) || isempty(spline_frs)
                    continue;
                end
                
                % Sanity check (and correct) all points on spline to be within env bounds. 
                total_spline1 = {cat(2,shared_spline{1},spline_opt{1}), ...
                                 cat(2,shared_spline{2},spline_opt{2}), ...
                                 cat(2,shared_spline{3},spline_opt{3}), ...
                                 cat(2,shared_spline{4},spline_opt{4}), ...
                                 cat(2,shared_spline{5},spline_opt{5})};
                total_spline2 = {cat(2,shared_spline{1},spline_frs{1}), ...
                                 cat(2,shared_spline{2},spline_frs{2}), ...
                                 cat(2,shared_spline{3},spline_frs{3}), ...
                                 cat(2,shared_spline{4},spline_frs{4}), ...
                                 cat(2,shared_spline{5},spline_frs{5})};    
                             
                %curr_spline = obj.sanity_check_spline(total_spline1, obj.num_waypts);
                
                % Compute the dynamically feasible horizon for the current plan.
                feasible_horizon1 = ...
                    obj.compute_dyn_feasible_horizon(total_spline1, ...
                                                  obj.max_linear_vel, ...
                                                  obj.max_angular_vel, ...
                                                  obj.horizon);
                feasible_horizon2 = ...
                    obj.compute_dyn_feasible_horizon(total_spline2, ...
                                                  obj.max_linear_vel, ...
                                                  obj.max_angular_vel, ...
                                                  obj.horizon); 
                                              
%                 hold on
%                 plot(shared_spline{1}, shared_spline{2}, 'm');
%                 plot(spline_opt{1}, spline_opt{2}, 'r');
%                 plot(spline_frs{1}, spline_frs{2}, 'b');
%                 xlim([-6.5,6.5]);
%                 ylim([-6.5,6.5]);
%                 grid on

                % If total plan for each contingency plan is dynamically
                % feasible, then evaluate reward. 
                if (feasible_horizon1 <= obj.horizon && ...
                        feasible_horizon2 <= obj.horizon)
                    
                    % Get reward of each trajectory segment
                    reward_shared = obj.eval_reward(shared_spline, 'all', 0, branch_t);
                    reward_opt = obj.eval_reward(spline_opt, 'opt', branch_t+obj.dt, obj.horizon);
                    reward_frs = obj.eval_reward(spline_frs, 'frs', branch_t+obj.dt, obj.horizon);
                    
                    % Compute total reward where we weight the reward
                    % contribution of contingency plans based on the
                    % likelihood of that goal occuring. 
                    %
                    %   R(traj) = R(shared) + \sum^M_i=1 b(g_i) * R(contingency_i)
                    %
                    reward = reward_shared + ...
                             obj.belief(1) * reward_opt + ...
                             obj.belief(2) * reward_frs;
                    
%                     hold on
%                     plot(shared_spline{1}, shared_spline{2}, 'r');
%                     plot(spline_g1{1}, spline_g1{2}, 'y');
%                     plot(spline_g2{1}, spline_g2{2}, 'b');
%                     xlim([-6.5,6.5]);
%                     ylim([-6.5,6.5]);
%                     grid on
                        
                    if (reward > opt_reward)
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
            
            for ti=1:length(obj.disc_xy) % length(obj.disc_xyth)
                candidate_goal = obj.disc_xy(ti, :); %obj.disc_xyth(ti, :);
                
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
                if (feasible_horizon <= hor)
                    
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
        function reward = eval_reward(obj, curr_spline, coll_check, start_t, hor)            
            xs = curr_spline{1};
            ys = curr_spline{2};
            ths = curr_spline{5};
            traj = [xs', ys'];
            
            % Find the time index to start and end the collision-checking 
            % with the human trajectories.
            [~,start_idx] = min(abs(obj.times - start_t));
            [~,end_idx] = min(abs(obj.times - hor));
            
            % TODO: add in penalty for orientation too?
            % TODO: add in penalty for human-driven vehicle prediction. 
            obs_r = eval_u(obj.g2d, obj.sd_obs, traj);
            goal_r = eval_u(obj.g2d, obj.sd_goal, traj);
            
            % for each time, compute if the robot state is in the human
            % state.
            % NOTE: ASSUMES THAT PREDICTION LENGTH AND TIME IS SAME AS
            % SPLINE PLAN LENGTH AND TIME.
            % circle_rad = 0.354;
            
            frs_relevant_preds = obj.frs_preds(1,start_idx:end_idx);
            opt_relevant_preds = obj.opt_preds(1,start_idx:end_idx);
            
            human_r = zeros(size(obs_r));
            for i=1:length(traj)
                r = 0.0;
                % compute the human reward.
                if strcmp(coll_check, 'all') || strcmp(coll_check, 'frs')
                    val = eval_u(obj.pred_g, frs_relevant_preds{i}, traj(i,:));
                    if val > obj.pthresh
                        r = -100.0;
                    end
                elseif strcmp(coll_check, 'opt')
                    val = eval_u(obj.pred_g, opt_relevant_preds{i}, traj(i,:));
                    if val > obj.pthresh
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
        
    end
end


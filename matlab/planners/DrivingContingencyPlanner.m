classdef DrivingContingencyPlanner < handle
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
        smaller_disc_xythvel
        gmin
        gmax
        gnums
        human_spline_g1
        human_spline_g2
        times
        dt
        belief
        circle_rad
        
    end
    
    methods
        %% Constructs Contingency Planner. 
        function obj = DrivingContingencyPlanner(num_waypts, horizon, ...
                                        goal, ...
                                        max_linear_vel, ...
                                        max_angular_vel, ...
                                        footprint_rad, ...
                                        sd_obs, ...
                                        sd_goal, ...
                                        g2d, ...
                                        g3d, ...
                                        gmin, gmax, gnums, ...
                                        circle_rad)
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
            obj.belief = [];
            obj.circle_rad = circle_rad;
            
            obj.times = linspace(0,obj.horizon,obj.num_waypts);
            obj.dt = obj.times(2) - obj.times(1); 
            
            gdisc = (gmax - gmin) ./ (gnums - 1);
            dv = 1;
            [X2D,Y2D] = meshgrid(gmin(1):gdisc(1):gmax(1), ...
                             gmin(2):gdisc(2):gmax(2));
            [X3D,Y3D,TH3D] = meshgrid(obj.g3d.min(1):obj.g3d.dx(1):obj.g3d.max(1), ...
                             obj.g3d.min(2):obj.g3d.dx(2):obj.g3d.max(2), ...
                             obj.g3d.min(3):obj.g3d.dx(3):obj.g3d.max(3));
            [X,Y,TH,V] = ndgrid(obj.g3d.min(1):obj.g3d.dx(1):obj.g3d.max(1), ...
                             obj.g3d.min(2):obj.g3d.dx(2):obj.g3d.max(2), ...
                             obj.g3d.min(3):obj.g3d.dx(3):obj.g3d.max(3), ...
                             0.01:dv:obj.max_linear_vel); 
            [XS, YS, THS, VS] = ndgrid(-5.7:obj.g3d.dx(1):3, ...    %-3.65:obj.g3d.dx(1):-1.825, ...
                                     -3.5:obj.g3d.dx(2):3, ...       %-3.65:obj.g3d.dx(2):0.5, ...
                                     obj.g3d.min(3):obj.g3d.dx(3):obj.g3d.max(3), ...
                                     1:dv:obj.max_linear_vel);%0.01:dv:obj.max_linear_vel);              
            obj.disc_xy = [X2D(:), Y2D(:)];
            obj.disc_xyth = [X3D(:), Y3D(:), TH3D(:)];
            obj.disc_xythvel = [X(:), Y(:), TH(:), V(:)];
            obj.smaller_disc_xythvel = [XS(:), YS(:), THS(:), VS(:)];
        end
        
        %% Helper function which pre-computes all shared goals
        %  which are dynamically feasible within the branching horizon. 
        %  (this is used to speed up the contingency planner parameter search) 
        function dyn_feas_xythvel = ...
                find_dyn_feas_shared_goal(obj, start, horiz, num_waypts)
            dyn_feas_xythvel = [];
            %for i=1:length(obj.smaller_disc_xythvel) 
            for i=1:length(obj.disc_xythvel)
                %candidate_goal = obj.smaller_disc_xythvel(i,:); 
                candidate_goal = obj.disc_xythvel(i,:);
                curr_spline = ...
                    spline(start, candidate_goal, horiz, num_waypts);
                feasible_horiz = obj.compute_dyn_feasible_horizon(curr_spline, ...
                                                  obj.max_linear_vel, ...
                                                  obj.max_angular_vel, ...
                                                  horiz);
                                              
                in_obs = (eval_u(obj.g2d, obj.sd_obs, candidate_goal(1:2)) < 0);
                if feasible_horiz <= horiz && ~in_obs
                    dyn_feas_xythvel = [dyn_feas_xythvel; candidate_goal];
                    
                    %hold on
                    %scatter(candidate_goal(1), candidate_goal(2));
                    %curr_spline = ...
                    %    spline(start, candidate_goal, horiz, num_waypts);
                    %feasible_horiz = obj.compute_dyn_feasible_horizon(curr_spline, ...
                    %              obj.max_linear_vel, ...
                    %              obj.max_angular_vel, ...
                    %              horiz);
                end
            end
        end
        
        %% Plans a contingency plan from start to goal.
        function opt_plan = ...
                contingency_plan(obj, start, goal, ...
                human_spline_g1, human_spline_g2, belief, branch_t)
            
            % Update the belief.
            obj.belief = belief;
            
            % Update the predictions. 
            obj.human_spline_g1 = human_spline_g1;
            obj.human_spline_g2 = human_spline_g2;
            
            % Compute number of waypoints in each part of the plan.
            [~,end_idx] = min(abs(obj.times - branch_t));
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
                                          
            % flag if there are no dynamically feasible shared goals!                              
            if isempty(feasible_shared_goals)
                error('No dynamically feasible shared goals! Try finer discretization.');
            end
                                          
            % Three options for how to search for shared goal:
            % Option 1:
            [num_shared_goals,~] = size(feasible_shared_goals); 
            % Option 2:
            %num_shared_goals = length(obj.disc_xythvel); 
            
            for i=1:num_shared_goals
                fprintf('Evaluating %d / %d (%f percent)...\n', ...
                    i, num_shared_goals, 100*(i /num_shared_goals));
                
                % Option 1: search only through precomputed dyn feasible
                % goals.
                shared_goal = feasible_shared_goals(i, :);
                
                % Option 2: full grid search over parameters.
                %shared_goal = obj.disc_xythvel(i, :);
                 
                % Compute shared spline. 
                shared_spline = spline(start, shared_goal, ...
                                        binned_branch_t, shared_num_waypts);

                % Extract final state of shared spline.
                start_branch = [shared_spline{1}(end), ...
                                shared_spline{2}(end), ...
                                shared_spline{5}(end), ...
                                shared_spline{3}(end)]; % (x,y,th,lin_v)        

                % Compute branching splines for g1 and g2   
                spline_g1 = obj.plan(start_branch, goal, binned_branch_t, ...
                                        binned_horiz, branch_num_waypts, 'g1');
                spline_g2 = obj.plan(start_branch, goal, binned_branch_t, ...
                                        binned_horiz, branch_num_waypts, 'g2');
                                    
                if isempty(spline_g1) || isempty(spline_g2)
                    continue;
                end
                
                % Sanity check (and correct) all points on spline to be within env bounds. 
                total_spline1 = {cat(2,shared_spline{1},spline_g1{1}(2:end)), ...
                                 cat(2,shared_spline{2},spline_g1{2}(2:end)), ...
                                 cat(2,shared_spline{3},spline_g1{3}(2:end)), ...
                                 cat(2,shared_spline{4},spline_g1{4}(2:end)), ...
                                 cat(2,shared_spline{5},spline_g1{5}(2:end))};
                total_spline2 = {cat(2,shared_spline{1},spline_g2{1}(2:end)), ...
                                 cat(2,shared_spline{2},spline_g2{2}(2:end)), ...
                                 cat(2,shared_spline{3},spline_g2{3}(2:end)), ...
                                 cat(2,shared_spline{4},spline_g2{4}(2:end)), ...
                                 cat(2,shared_spline{5},spline_g2{5}(2:end))};    
                
                % Sanity check (and correct) all points on spline to be within env bounds. 
                total_spline1 = obj.sanity_check_spline(total_spline1, obj.num_waypts);
                total_spline2 = obj.sanity_check_spline(total_spline2, obj.num_waypts);
                
                % Compute the dynamically feasible horizon for the current plan.
                feasible_horizon1 = ...
                    obj.compute_dyn_feasible_horizon(total_spline1, ...
                                                  obj.max_linear_vel, ...
                                                  obj.max_angular_vel, ...
                                                  binned_horiz-binned_branch_t);
                feasible_horizon2 = ...
                    obj.compute_dyn_feasible_horizon(total_spline2, ...
                                                  obj.max_linear_vel, ...
                                                  obj.max_angular_vel, ...
                                                  binned_horiz-binned_branch_t);  
                                              
                % If total plan for each contingency plan is dynamically
                % feasible, then evaluate reward. 
                if (feasible_horizon1 <= binned_horiz && ...
                        feasible_horizon2 <= binned_horiz)
                    
                    % Get reward of each trajectory segment
                    reward_shared = obj.eval_reward(shared_spline, 'all', 0, binned_branch_t);
                    reward_g1 = obj.eval_reward(spline_g1, 'g1', binned_branch_t, binned_horiz);
                    reward_g2 = obj.eval_reward(spline_g2, 'g2', binned_branch_t, binned_horiz);
                    
                    % Get goal-based reward at the end of the trajectory.
                    traj_g1 = [spline_g1{1}', spline_g1{2}'];
                    traj_g2 = [spline_g2{1}', spline_g2{2}'];
                    goal_r_g1 = eval_u(obj.g2d, obj.sd_goal, traj_g1(end-1:end,:));
                    goal_r_g2 = eval_u(obj.g2d, obj.sd_goal, traj_g2(end-1:end,:));
                    
                    % Acceleration penalty at the branching point. 
                    vels_g1 = [shared_spline{3}, spline_g1{3}];
                    vels_g2 = [shared_spline{3}, spline_g2{3}];
                    accel_g1 = (vels_g1(2:end) - vels_g1(1:end-1))/obj.dt;
                    accel_g2 = (vels_g2(2:end) - vels_g2(1:end-1))/obj.dt;
                    low_idx = shared_num_waypts-3;
                    up_idx = shared_num_waypts+3;
                    if low_idx < 1
                        low_idx = 1;
                    end
                    if up_idx > length(vels_g1)
                        up_idx = length(vels_g1);
                    end
                    accel_around_branch_t_g1 = accel_g1(low_idx:up_idx);
                    accel_around_branch_t_g2 = accel_g2(low_idx:up_idx);
                    accel_r_g1 = 0.0; %-10.0 * norm(accel_around_branch_t_g1);  
                    accel_r_g2 = 0.0; %-10.0 * norm(accel_around_branch_t_g2); 
                    
                    % Compute total reward where we weight the reward
                    % contribution of contingency plans based on the
                    % likelihood of that goal occuring. 
                    %
                    %   R(traj) = R(shared) + \sum^M_i=1 b(g_i) * R(contingency_i)
                    %
                    reward = reward_shared + ...
                             obj.belief(1) * (reward_g1 + sum(goal_r_g1) + accel_r_g1) + ...
                             obj.belief(2) * (reward_g2 + sum(goal_r_g2) + accel_r_g2); 
                         
                    if (reward > opt_reward)
                        opt_reward = reward;
                        opt_plan = {shared_spline, spline_g1, spline_g2};
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
        %                           against. options: 'all', 'g1', 'g2') 
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
                compute_dyn_feasible_horizon(obj, curr_spline, ...
                                              max_linear_vel, ...
                                              max_angular_vel, ...
                                              final_t)
                                          
              % NOTE: here we upsample the spline to make sure 
              % we catch dynamic infeasibility!!   
              %curr_start = [curr_spline{1}(1), curr_spline{2}(1), ...
              %               curr_spline{5}(1), curr_spline{3}(1)];
              %curr_goal = [curr_spline{1}(end), curr_spline{2}(end), ...
              %              curr_spline{5}(end), curr_spline{3}(end)];
              
              % Upsample by 3x. 
              %upsamp_waypts = (length(curr_spline{1})-1)*2 + 1; 
              %upsamp_spline = spline(curr_start, curr_goal, final_t, upsamp_waypts);
              %original_spline = spline(curr_start, curr_goal, final_t, length(curr_spline{1}));
              
              % Quick n dirty hack. 
              angles = curr_spline{5};
              ang_vel_approx = (angles(2:end) - angles(1:end-1)) ./ obj.dt;
              
              % Compute max linear and angular speed.
              plan_max_lin_vel = max(curr_spline{3});
              plan_max_angular_vel = max(abs(curr_spline{4}));
              
              max_ang_vel_approx = max(abs(ang_vel_approx));
              
              if max_ang_vel_approx > max_angular_vel
                  plan_max_angular_vel = max_ang_vel_approx;
              end
              
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
            vels = curr_spline{3};
            ths = curr_spline{5};
            traj = [xs', ys'];
            traj_theta = [xs', ys', ths'];
            
            % Find the time index to start and end the collision-checking 
            % with the human trajectories.
            [~,start_idx] = min(abs(obj.times - start_t));
            [~,end_idx] = min(abs(obj.times - hor));
            
            % TODO: add in penalty for orientation too?
            % TODO: add in penalty for human-driven vehicle prediction. 
            obs_r = eval_u(obj.g2d, obj.sd_obs, traj);
            
            % --> option 1: evaluate goal reward over all states.
            %goal_r = eval_u(obj.g2d, obj.sd_goal, traj);
            % --> option 2: evaluate goal reward only over last state(s).
            goal_r = 0.0; %eval_u(obj.g2d, obj.sd_goal, traj(end-1:end,:));
            
            % for each time, compute if the robot state is in the human
            % state.
            % NOTE: ASSUMES THAT PREDICTION LENGTH AND TIME IS SAME AS
            % SPLINE PLAN LENGTH AND TIME.
            
            hg1_traj = [obj.human_spline_g1{1}', obj.human_spline_g1{2}'];
            hg2_traj = [obj.human_spline_g2{1}', obj.human_spline_g2{2}'];
            
            %[ur, ul, dl, dr] = ....
            %    obj.get_four_corners_rot_rect(xc, yc, th, car_len, car_width);
            %[d, ~] = signedDistancePolygons(four_corners_box1, four_corners_box2);
            
            diff_hg1 = traj - hg1_traj(start_idx:end_idx,:);
            diff_hg2 = traj - hg2_traj(start_idx:end_idx,:);
            
            % do collision checking between the body of the robot and human 
            % vehicle which are both approximated by circles w/radius circle_rad
            d_to_hg1 = sqrt(diff_hg1(:,1).^2 + ...
                            diff_hg1(:,2).^2) - ...
                            (obj.circle_rad + obj.circle_rad); 
            d_to_hg2 = sqrt(diff_hg2(:,1).^2 + ...
                            diff_hg2(:,2).^2) - ...
                            (obj.circle_rad + obj.circle_rad); 
            
            % compute the human reward.
            if strcmp(coll_check, 'all')
                human_r = (d_to_hg1 <= 0) .* -100.0 + ...
                            (d_to_hg2 <= 0) .* -100.0 + ...
                            (d_to_hg1 > 0) .* 0.0 + ...
                            (d_to_hg2 > 0) .* 0.0;
            elseif strcmp(coll_check, 'g1')
                human_r = (d_to_hg1 <= 0) .* -100.0 + ...
                            (d_to_hg1 > 0) .* 0.0;
            elseif strcmp(coll_check, 'g2')
                human_r = (d_to_hg2 <= 0) .* -100.0 + ...
                            (d_to_hg2 > 0) .* 0.0;     
            else
                error('Invalid collision check option! %s', coll_check);
            end
                    
            %if human_r ~= 0
            %    fprintf("collision!\n")
            %end
            
            reward = sum(obs_r + human_r) + sum(goal_r); 
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


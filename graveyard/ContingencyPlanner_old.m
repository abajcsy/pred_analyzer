classdef ContingencyPlanner < handle
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
        gmin
        gmax
        gnums
        human_spline_g1
        human_spline_g2
        times
        dt
    end
    
    methods
        %% Constructs Contingency Planner. 
        function obj = ContingencyPlanner(num_waypts, horizon, ...
                                        goal, ...
                                        max_linear_vel, ...
                                        max_angular_vel, ...
                                        footprint_rad, ...
                                        sd_obs, ...
                                        sd_goal, ...
                                        human_spline_g1, ...
                                        human_spline_g2, ...
                                        g2d, ...
                                        g3d, ...
                                        gmin, gmax, gnums)
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
            obj.human_spline_g1 = human_spline_g1;
            obj.human_spline_g2 = human_spline_g2;
            
            obj.times = linspace(0,obj.horizon,obj.num_waypts);
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
        
        %% Plans a contingency plan from start to goal.
        function [shared_spline, spline_g1, spline_g2] = ...
                contingency_plan(obj, start, goal, branch_t)
            
            [~,end_idx] = min(abs(obj.times - branch_t));
            
            % Compute shared spline. 
            shared_num_waypts = end_idx;
            shared_goal = [goal(1:3), 0.01]; 
            fprintf("Computing shared spline...\n"); 
            shared_spline = obj.plan(start, shared_goal, 0, branch_t, ...
                                        shared_num_waypts, 'all');
            
            % Extract final state of shared spline.
            start_branch = [shared_spline{1}(end), ...
                            shared_spline{2}(end), ...
                            shared_spline{5}(end), ...
                            shared_spline{3}(end)]; % (x,y,th,lin_v)        
                        
            % Compute branching splines for g1 and g2   
            branch_num_waypts = abs(obj.num_waypts - end_idx);
            fprintf("Computing spline for g1 hypothesis...\n"); 
            spline_g1 = obj.plan(start_branch, goal, branch_t+obj.dt, ...
                                    obj.horizon, branch_num_waypts, 'g1');
            fprintf("Computing spline for g2 hypothesis...\n");  
            spline_g2 = obj.plan(start_branch, goal, branch_t+obj.dt, ...
                                    obj.horizon, branch_num_waypts, 'g2');
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
            figure
            all_rewards = [];
            plt_handles = {};
            
            for ti=1:length(obj.disc_xy) % length(obj.disc_xyth)
                candidate_goal = obj.disc_xy(ti, :); %obj.disc_xyth(ti, :);
                
                % ignore candidate goals inside obstacles.
                if eval_u(obj.g2d, obj.sd_obs, candidate_goal(1:2)) < 0
                    continue;
                end
                
                % orientation should match with goal final vel ~= 0.
                candidate_goal = [candidate_goal, goal(3), 0.01]; %[candidate_goal, 0.01]; 
                
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
                        
                    if strcmp(coll_check, 'g2') && (ti == 280 || ti == 1918) %(ti == 240 || ti == 241)
                        bla = 1;
                    end
                    
                    reward = obj.eval_reward(curr_spline, coll_check, start_t, hor);
                        
                    if (reward > opt_reward)
                        opt_reward = reward;
                        opt_spline = curr_spline;
                        
%                         if strcmp(coll_check, 'g2') 
%                             % blaaa
%                             hold on
%                             contour(obj.g2d.xs{1}, obj.g2d.xs{2}, obj.sd_obs, [0,0]);
%                             colors = [linspace(0,1,length(curr_spline{1}))', ...
%                                         zeros([length(curr_spline{1}), 1]), ...
%                                         zeros([length(curr_spline{1}), 1])];
%                             p = plot(curr_spline{1}, curr_spline{2});
%                             xlim([obj.gmin(1),obj.gmax(1)]);
%                             ylim([obj.gmin(2),obj.gmax(2)]);
%                             all_rewards(end+1) = reward;
%                             plt_handles{end+1} = p;
%                             grid on
%                             % blaaa
%                         end
                        
                    end
                end
            end
            
%             c = linspace(0.1,0.9,length(plt_handles));
%             for i=1:length(plt_handles)
%                 plt_handles{i}.Color = [c(i),0,0];
%                 plt_handles{i}.LineWidth = 2;
%             end
%             p = plot(opt_spline{1}, opt_spline{2}, 'Color', 'b', 'LineWidth', 2);
            
            if isempty(opt_spline)
                error("Unable to find dynamically feasible and low cost spline plan!");
            end
            
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
            traj_theta = [xs', ys', ths'];
            
            % Find the time index to start and end the collision-checking 
            % with the human trajectories.
            [~,start_idx] = min(abs(obj.times - start_t));
            [~,end_idx] = min(abs(obj.times - hor));
            
            % TODO: add in penalty for orientation too?
            % TODO: add in penalty for human-driven vehicle prediction. 
            obs_r = eval_u(obj.g2d, obj.sd_obs, traj);
            goal_r = eval_u(obj.g2d, obj.sd_goal, traj);
            % goal_r = eval_u(obj.g3d, obj.sd_goal, traj_theta);
            
            % for each time, compute if the robot state is in the human
            % state.
            % NOTE: ASSUMES THAT PREDICTION LENGTH AND TIME IS SAME AS
            % SPLINE PLAN LENGTH AND TIME.
            car_len = 3; %4.5; % in m
            car_width = 1.2; %1.8; % in m
            circle_rad = 1.2;
            
            hg1_traj = [obj.human_spline_g1{1}', obj.human_spline_g1{2}'];
            hg2_traj = [obj.human_spline_g2{1}', obj.human_spline_g2{2}'];
            
            %[ur, ul, dl, dr] = ....
            %    obj.get_four_corners_rot_rect(xc, yc, th, car_len, car_width);
            %[d, ~] = signedDistancePolygons(four_corners_box1, four_corners_box2);
            
            diff_hg1 = traj - hg1_traj(start_idx:end_idx,:);
            diff_hg2 = traj - hg2_traj(start_idx:end_idx,:);
            
            d_to_hg1 = sqrt(diff_hg1(:,1).^2 + ...
                            diff_hg1(:,2).^2) - ...
                            (circle_rad + circle_rad);
            d_to_hg2 = sqrt(diff_hg2(:,1).^2 + ...
                            diff_hg2(:,2).^2) - ...
                            (circle_rad + circle_rad);
            
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


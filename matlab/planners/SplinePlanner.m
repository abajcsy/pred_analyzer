classdef SplinePlanner
    %SPLINEPLANNER Summary of this class goes here
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
        disc_xy
        gmin
        gmax
        gnums
    end
    
    methods
        %% Constructs Spline Planner. 
        function obj = SplinePlanner(num_waypts, horizon, ...
                                        goal, ...
                                        max_linear_vel, ...
                                        max_angular_vel, ...
                                        footprint_rad, ...
                                        sd_obs, ...
                                        sd_goal, ...
                                        g2d, ...
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
            obj.gmin = gmin;
            obj.gmax = gmax;
            obj.gnums = gnums;
            
            gdisc = (gmax - gmin) ./ (gnums - 1);
            [X,Y] = meshgrid(gmin(1):gdisc(1):gmax(1), ...
                             gmin(2):gdisc(2):gmax(2));
            obj.disc_xy = [X(:), Y(:)];
        end
        
        %% Plans a path from start to goal. 
        function opt_spline = plan(obj, start, goal)
            opt_reward = -100000000000000;
            opt_spline = {};
            curr_spline = {};
            
            for ti=1:length(obj.disc_xy)
                candidate_goal = obj.disc_xy(ti, :);
                % orientation should match with goal final vel ~= 0.
                candidate_goal = [candidate_goal, goal(3), 0.01]; 
                
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
                    reward = obj.eval_reward(curr_spline);
                    
                    if (reward > opt_reward)
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
        function reward = eval_reward(obj, curr_spline)            
            xs = curr_spline{1};
            ys = curr_spline{2};
            traj = [xs', ys'];
            
            % TODO: add in penalty for orientation too?
            % TODO: add in penalty for human-driven vehicle prediction. 
            obs_r = eval_u(obj.g2d, obj.sd_obs, traj);
            goal_r = eval_u(obj.g2d, obj.sd_goal, traj);
            reward = sum(obs_r + goal_r); 
        end
        
    end
end


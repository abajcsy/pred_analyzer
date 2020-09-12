classdef MDPHumanSGD3D < handle

    properties
        num_ctrls       % (int) number of discrete controls
        controls        % (arr) discretized controls
        z0              % (cell) initial joint state z = (x, y, theta)
        uThresh         % (float) threshold probability for sufficiently likely controls
        trueTheta       % (float) true human reward weight.
        v_funs          % (cell) Value functions (one entry per theta)
        q_funs          % (cell) Q-functions (one entry per theta)
        reward_info     % (struct) Information for reward function construction
        nearInf         % (float) Very large number which is < Inf 
        has_obs         % (bool) If this environment has obstacles or not.
        gdisc           % (arr) discretization resolution in x,y,theta
        alpha           % (float) step size for SGD update
        w1              % (float) weight on distance-to-goal (pre-specified)
        
        goal_feature    % (Map) phi(x,u) for distance-to-goal feature
        obs_feature     % (Map) phi(x,u) for distance-to-obstacles feature
    end
    
    methods
        function obj = MDPHumanSGD3D(z0, reward_info, trueTheta, ...
                                        uThresh, gdisc, gamma, ...
                                        eps, alpha, w1)
            % MDPHumanSGD3D 
            %   Represents joint dynamics of a 2D point human 
            %   and an unknown reward parameter. The reward parameter 
            %   is denoted by theta and weights how far from obstacles the 
            %   human prefers to stay. The physical dynamics follow 
            %   an MDP grid structure where actions are UP/DOWN/LEFT/RIGHT
            %   and the four DIAGONALS and STOP. 
            %
            %   State:
            %       z = [x, y, theta]
            %           x \in [xmin, xmax]          (1D)
            %           y \in [ymin, ymax]          (1D)
            %           theta \in [thmin, thmax]    (1D)
            %   Dynamics:
            %       zdot = [x + u, 
            %               y + u, 
            %               theta + alpha * \nabla_theta Q(x,u;theta)]
            %       where:
            %           \nabla_theta Q(x,u;theta) 
            %           is gradient of Q wrt theta for a given (x,u) pair. 
            %
            %       Q(x,u,theta) is computed via value iteration and the 
            %       reward function information packed into reward_info.
            %       The reward function is structured as a linear
            %       combination of features:
            %           r(x,u) = w^T * phi(x,u)
            %       where:
            %           phi(x,u) = [dist_to_goal(x,u), dist_to_obs(x,u)]    
            %           w = [1 ; theta]
            %       encoding a known weight to the goal, but unknown weight
            %       to stay away from obstacles. 
            % 
            %   Controls:
            %       u \in {'S', 'D', 'U', 'L', 'LD', 'LU', 'R', 'RD', 'RU'}
            %   
            %   Inputs:
            %       z0: (cell) joint initial condition 
            %       reward_info: (struct) reward information
            %                   .obstacles: axis-aligned bounds of obs
            %                   .g: grid for the physical states
            %                   .thetas: list of discrete thetas for which
            %                            to compute Q-function
            %                   .goal: goal location in physical state
            %                          space
            %                   .goalRad: radius around goal for distance
            %                             function
            %       trueTheta: (cell) true reward weight
            %       uThresh: (float) threshold for likely controls
            %       gdisc: (arr) discretization resolution in x,y,theta
            %       gamma: (float) discount factor for Q-function compute
            %       eps: (float) convergence threshold for Q-function 
            %       alpha: (float) step size for SGD update
            % 
            obj.controls = obj.generate_controls_mdp(gdisc);
            obj.num_ctrls = numel(obj.controls);
            obj.z0 = z0;
            obj.uThresh = uThresh;
            obj.trueTheta = trueTheta;
            obj.nearInf = 1000000000.0;
            obj.v_funs = cell(1,numel(reward_info.thetas));
            obj.q_funs = cell(1,numel(reward_info.thetas));
            obj.reward_info = reward_info;
            obj.gdisc = gdisc;
            obj.alpha = alpha;
            obj.w1 = w1;
            
            obj.has_obs = false;
            if isfield(obj.reward_info, 'obstacles')
                obj.has_obs = true;
            end
            
            % Pre-compute the reward features. 
            %       phi(x,u) = [dist_to_goal(x,u), dist_to_obs(x,u)]    
            obj.goal_feature = obj.compute_goal_feature(reward_info);
            obj.obs_feature = obj.compute_obs_feature(reward_info);
            
            % Pre-compute the Q-functions for a discrete set of thetas
            % (used for finite-difference approx to gradient) 
            for th_idx=1:numel(reward_info.thetas)
                fprintf('Pre-computing Q-function for theta = %d / %d\n', ...
                    th_idx, numel(reward_info.thetas));
                st = tic;
                [v_f, q_f] = obj.compute_q_fun(th_idx, gamma, eps);
                obj.v_funs{th_idx} = v_f;
                obj.q_funs{th_idx} = q_f;
                et = toc(st);
                fprintf('   compute time: %f s\n', et);
            end
        end
        
        %% Joint dynamics zdot = [xdot, bdot].
        function znext = dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(size(z));
            znext{1} = z{1} + u(1);
            znext{2} = z{2} + u(2);
            znext{3} = obj.sgd(u,z);
        end
        
        %% Dyanmics of physical states.
        function znext = physical_dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(2);
            znext{1} = z{1} + u(1);
            znext{2} = z{2} + u(2);
        end
        
        %% Performs SGD update on weight parameter
        %   theta_n = theta_n-1 + \alpha * \nabla_theta Q(x,u,theta_n-1)
        %  Uses a finite-difference approximation to \nabla_theta Q
        function theta_n = sgd(u,z)
            theta = z{3};
            dth = obj.gdisc(3);
            
            % Approximate the gradient:
            % \partial Q(x,u,theta_n)/\partial theta_n ~= 
            %           Q(x,u; theta_n(1) + dth) - Q(x,u;theta_n(1))/dth
            Q_theta = obj.q_funs{th_idx}(num2str(u));
            Q_theta_dth = obj.q_funs{th_idx}(num2str(u));
            
%             q_val.GetDataAtReal(z);
            
            % Compute: \nabla_theta Q(x,u,theta_n-1)
            dQ = (Q_theta_dth - Q_theta) / dth;
            
            % Perform SGD update:
            theta_n = theta + obj.alpha .* dQ;
        end
        
        %% Generates the discrete controls for the MDP human. 
        function controls = generate_controls_mdp(obj, gdisc)
            controls = cell(1,9);
            xs = {0, -1*gdisc(1), gdisc(1)};
            ys = {0, -1*gdisc(2), gdisc(2)};

            ind = 1;
            for i=1:numel(xs)
                for j=1:numel(ys)
                    controls{ind} = [xs{i}, ys{j}];
                    ind = ind + 1;
                end
            end
        end
        
        %% Computes phi(x,u) for distance-to-goal feature
        % Returns map where key is control and value is grid over physical
        % state space with the goal-feature for each (x,y) pair.
        function goal_feature = compute_goal_feature(obj, reward_info)
            % Note: need to negate goal signed distance. 
            goal_feature = -1 * shapeCylinder(reward_info.g, [], ...
                                  reward_info.goal, reward_info.goalRad);
        end
        
        %% Computes phi(x,u) for distance-to-obs feature
        % Returns map where key is control and value is grid over physical
        % state space with the obstacle-feature for each (x,y) pair.
        function obs_feature = compute_obs_feature(obj, reward_info)
            obs_feature = [];
            for oi = 1:length(reward_info.obstacles)
                    obs_info = reward_info.obstacles{oi};
                    obs_min = obs_info(1:2);
                    obs_max = obs_info(1:2) + obs_info(3:4);
                    obs_feature_curr = ...
                        shapeRectangleByCorners(reward_info.g, ...
                                                obs_min, obs_max);
                    % combine all obstacle signed distance functions.
                    if isempty(obs_feature)
                        obs_feature = obs_feature_curr;
                    else
                        obs_feature = min(obs_feature, obs_feature_curr);
                    end
            end
        end
        
        %% Computes the Q-function for theta denoted by true_theta_idx 
        %   via value iteration. 
        function [v_grid, q_fun] = compute_q_fun(obj, true_theta_idx, gamma, eps)
            % Extract reward info.
            r_fun = containers.Map;
            state_grid = obj.reward_info.g.xs;
            g_min = obj.reward_info.g.min';
            g_max = obj.reward_info.g.max';
            g_nums = obj.reward_info.g.N';
            
            % Setup reward weights.
            w2 = obj.reward_info.thetas{true_theta_idx};
  
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                next_state = obj.physical_dynamics(state_grid, u_i);
                
                % Make next states into a matrix. 
                next_points = [next_state{1}(:), next_state{2}(:)];
                
                % Get phi(x,u) at the next state after applying u
                phi_g = eval_u(obj.reward_info.g, obj.goal_feature, next_points);
                phi_o = eval_u(obj.reward_info.g, obj.obs_feature, next_points);
                
                % Set reward to -inf if moving outside grid
                phi_g(isnan(phi_g)) = -obj.nearInf;
                phi_o(isnan(phi_o)) = -obj.nearInf;
                
                phi_g = reshape(phi_g, g_nums(1), g_nums(2));
                phi_o = reshape(phi_o, g_nums(1), g_nums(2));

                % Compute: r(x,u) = w^T * phi(x,u)
                %   where the reward is -inf if the action takes the
                %   agent outside of the grid and = 0 else.
                r_i = obj.w1 .* phi_g + w2 .* phi_o;
                r_fun(num2str(u_i)) = r_i;
                
                % === plot reward landscpe === %
                % surf(obj.reward_info.g.xs{1},obj.reward_info.g.xs{2}, r_i)
                % colorbar
                % === plot reward landscpe === %
            end
            
            % Compute value function
            v_fun_prev = zeros(g_nums); %rand(obj.reward_info.gnums);
            v_grid = Grid(g_min, g_max, g_nums);
            while true
                v_grid.SetData(v_fun_prev);
                possible_vals = zeros([g_nums, numel(obj.controls)]);
                for i=1:numel(obj.controls)
                    u_i = obj.controls{i};
                    next_state = obj.physical_dynamics(state_grid, u_i);
                    v_next = v_grid.GetDataAtReal(next_state);
                    possible_vals(:,:,i) = r_fun(num2str(u_i)) + (gamma .* v_next);
                end
                v_fun = max(possible_vals, [], 3);
                
                % \ell_\infty stopping condition
                v_delta = v_fun - v_fun_prev;
                max_dev = max(abs(v_delta),[],'all');
                if max_dev < eps
                    break
                else
                    v_fun_prev = v_fun;
                end
            end
            v_grid.SetData(v_fun);
            
            % Compute Q-function
            q_fun = containers.Map;
            sum_exp = 0;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                next_state = obj.physical_dynamics(state_grid, u_i);
                v_next = v_grid.GetDataAtReal(next_state);
                q_data = r_fun(num2str(u_i)) + (gamma .* v_next);
                q_i = Grid(g_min, g_max, g_nums);
                q_i.SetData(q_data);
                q_fun(num2str(u_i)) = q_i;
                sum_exp = sum_exp + exp(q_i.data);
            end
            
            % Make invalid states have Q-value of zero. States are invalid
            % if \sum e^Q(x,u') == 0 (i.e no states give a non-zero action)
%             isInvalid = (maxQ <= -obj.nearInf);
            isInvalid = (sum_exp == 0);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_i = q_fun(num2str(u_i)).data;
                q_i(isInvalid) = 0;
                q = Grid(g_min, g_max, g_nums);
                q.SetData(q_i);
                q_fun(num2str(u_i)) = q;
            end
        end

        %% Plots optimal policy for the inputted theta. 
        function plot_opt_policy(obj, theta_idx)
            q_fun = obj.q_funs{theta_idx};
            all_q_vals = zeros([obj.reward_info.g.N', numel(obj.controls)]);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_i = q_fun(num2str(u_i)).data;
                all_q_vals(:,:,i) = q_i;
            end
            [opt_vals, opt_u_idxs] = max(all_q_vals, [], 3);
            
            figure
            hold on
            % plot the optimal action at each state.
            surf(obj.reward_info.g.xs{1}, obj.reward_info.g.xs{2}, opt_u_idxs);
            xlabel('x');
            ylabel('y');
            colorbar
            
            % plot obstacle.
            if obj.has_obs
                for oi = 1:length(obj.reward_info.obstacles)
                    obs_info = obj.reward_info.obstacles{oi};
                    obs_min = obs_info(1:2);
                        
                    x_min = obs_min(1);
                    y_min = obs_min(2);
                    p_min = 1;
                    l = [obs_info(3), ...
                        obs_info(4), ...
                        9];
                    plotcube(l,[x_min y_min p_min], .5, [1 1 1]);
                end
            end
            
            view(0,90);
            title(strcat("Optimal policy for theta=", num2str(theta_idx),". Color at state => opt control"));
        end
        
        %% Plots optimal policy for the inputted theta. 
        function plot_opt_policy_from_x0(obj, xinit, theta_idx)
            q_fun = obj.q_funs{theta_idx};
            all_q_vals = zeros([obj.reward_info.g.N', numel(obj.controls)]);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_i = q_fun(num2str(u_i)).data;
                all_q_vals(:,:,i) = q_i;
            end
            [opt_vals, opt_u_idxs] = max(all_q_vals, [], 3);
            
            % Colors for each theta
            num_thetas = length(obj.reward_info.thetas);
            scale = obj.gdisc(1);
            
            traj_start_color = [176, 0, 0]/255;
            traj_end_color = [0, 106, 176]/255; 
            rs = linspace(traj_start_color(1), traj_end_color(1), num_thetas);
            gs = linspace(traj_start_color(2), traj_end_color(2), num_thetas);
            bs = linspace(traj_start_color(3), traj_end_color(3), num_thetas);
            
            traj_color = [rs(theta_idx), gs(theta_idx), bs(theta_idx)];
            
            figure
            hold on
            uopts = Grid(obj.reward_info.g.min, ...
                        obj.reward_info.g.max, ...
                        obj.reward_info.g.N);
            uopts.SetData(opt_u_idxs);
            xcurr = xinit;
            opt_traj = [[xcurr{1};xcurr{2}]];
            for t=1:10
                % plot the current state.
                scatter(xcurr{1}, ...
                        xcurr{2}, ...
                        'linewidth', 2, ...
                        'marker', 'o', ...
                        'markeredgecolor', traj_color, ...
                        'markerfacecolor', traj_color);
                % get optimal control at this state
                uopt_idx = uopts.GetDataAtReal(xcurr);
                xcurr = obj.physical_dynamics(xcurr, obj.controls{uopt_idx});
                opt_traj(:,end+1) = [xcurr{1};xcurr{2}];
            end 
            
            % Plot lines connecting each of the state dots. 
            plot(opt_traj(1,:), opt_traj(2,:), 'Color', traj_color, 'linewidth', 2);
            
            % Goal colors.
            g1Color = 'r';
            g2Color = [38., 138., 240.]/255.;
    
            % g1
            scatter(obj.reward_info.goal(1), ...
                obj.reward_info.goal(2), ...
                'linewidth', 2, ...
                'marker', 'o', ...
                'markeredgecolor', g1Color, ...
                'markerfacecolor', g1Color);
            g1Txt = 'g1';
            t1 = text(obj.reward_info.goal(1)-0.05, ...
                        obj.reward_info.goal(2)-0.3, ...
                        0.55, g1Txt);
            t1.FontSize = 11;
            t1.Color = g1Color;
      
            % plot obstacle.
            if obj.has_obs
                for oi = 1:length(obj.reward_info.obstacles)
                    obs_info = obj.reward_info.obstacles{oi};
                    obs_min = obs_info(1:2);
                        
                    x_min = obs_min(1);
                    y_min = obs_min(2);
                    p_min = 1;
                    l = [obs_info(3), ...
                        obs_info(4), ...
                        9];
                    plotcube(l,[x_min y_min p_min], .5, [0.3 0.3 0.3]);
                end
            end
            set(gcf,'color','w');
            xlim([obj.reward_info.g.min(1), obj.reward_info.g.max(1)]);
            ylim([obj.reward_info.g.min(2), obj.reward_info.g.max(2)]);
            set(gcf, 'Position', [0,0,800,800])
            box on
            grid off
            view(0,90);
            title(strcat("Optimal policy for theta=", num2str(theta_idx),"."));
        end
        
    end
end


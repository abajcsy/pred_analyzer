classdef MDPHumanConfidence3D < handle
    
    properties
        thetas          % (cell) set of model parameters
        num_ctrls       % (int) number of discrete controls
        controls        % (arr) discretized controls
        z0              % (cell) initial joint state z = (x, y, b(theta_1))
        uThresh         % (float) threshold probability for sufficiently likely controls
        trueBetaIdx    % (int) true human intent parameter.
        v_fun          % Value functions for theta
        q_fun          % Q-functions for theta
        reward_info     % (struct) Information for reward function construction
        nearInf         % (float) Very large number which is < Inf 
        has_obs         % (bool) If this environment has obstacles or not.
        betas            % (cell) array of 3 beta values \in [0, inf) which governs variance of likelihood model.
        gdisc           % (arr) discretization resolution in x,y,b(th=1)
        b_range         % (arr) range of belief values to clip to.
        
        % Note: Is it safe to assume b scalar or can we have len(theta)>2
        % (which would mean to have b of dimension len(theta)-1)?
    end
    
    methods
        function obj = MDPHumanConfidence3D(z0, reward_info, trueBetaIdx, ...
                                        uThresh, gdisc, gamma, eps, betas, b_range)
            %   Represents joint dynamics of a 2D point human 
            %   and a 2D discrete belief distribution. The belief is over
            %   two unknown goals, denoted by theta. The dynamics follow 
            %   an MDP grid structure where actions are UP/DOWN/LEFT/RIGHT
            %   and the four DIAGONALS and STOP. 
            %
            %   State:
            %       z = [x, y, b(beta = beta_1)]
            %   Dynamics:
            %       zdot = [x + u, 
            %               y + u, 
            %               P(u | x, beta_1)b(beta = b1)/P(x,u)
            %       where:
            %           P(u | x, beta) \propto e^{beta * Q(x,u)}
            %
            %       Q(x,u) is computed via value iteration and the 
            %       reward function information packed into reward_info.
            %   Controls:
            %       u \in {'S', 'D', 'U', 'L', 'LD', 'LU', 'R', 'RD', 'RU'}
            %   
            obj.controls = obj.generate_controls_mdp(gdisc);
            obj.num_ctrls = numel(obj.controls);
            obj.z0 = z0;
            obj.uThresh = uThresh;
            obj.trueBetaIdx = trueBetaIdx;
            obj.nearInf = 1000000000.0;
            obj.reward_info = reward_info;
            obj.betas = betas;
            obj.gdisc = gdisc;
            obj.b_range = b_range;
            
            obj.has_obs = false;
            if isfield(obj.reward_info, 'obstacles')
                obj.has_obs = true;
            end
            
            % Pre-compute the Q-functions for each theta.
            fprintf('Pre-computing Q-function for fixed theta (goal)...\n');
            st = tic;
            [v_f, q_f] = obj.compute_q_fun(gamma, eps);
            obj.v_fun = v_f;
            obj.q_fun = q_f;
            et = toc(st);
            fprintf('   compute time: %f s\n', et);
        end
        
        %% Joint dynamics zdot = [xdot, bdot].
        function znext = dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(size(z));
            znext{1} = z{1} + u(1);
            znext{2} = z{2} + u(2);
            beta_posteriors = obj.belief_update(u,z);
            znext{3} = beta_posteriors{1};
        end
        
        %% Dyanmics of physical states.
        function znext = physical_dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(2);
            znext{1} = z{1} + u(1);
            znext{2} = z{2} + u(2);
        end
        
        %% Dynamics of the belief states (i.e. belief update)
        function beta_posteriors = belief_update(obj,u,z)
            % Calculate Bayesian posterior update.
            b1 = obj.pugivenxbeta(u, z, obj.betas{1}) .* z{3};
            b2 = obj.pugivenxbeta(u, z, obj.betas{2}) .* (1-z{3});
            normalizer = b1 + b2;
            
            beta_posteriors = cell(1,1);
            % Is min(max(...)) correct for num beta > 2?
            beta_posteriors{1} = min(max(b1 ./ normalizer, obj.b_range(1)), obj.b_range(2));
        end
        
        %% Mask over states for each control denoting if that control 
        %  is sufficiently likely at this state. 
        function likelyMasks = getLikelyMasks(obj, z)
            likelyMasks = containers.Map;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};

                % We want to choose controls such that:
                %   U = {u : P(u | x, theta = trueTheta) > delta}
                putheta = obj.pugivenxbeta(u_i, z, obj.betas{obj.trueBetaIdx});
%                 max(putheta,[],'all')
                mask = (putheta > obj.uThresh);
                mask = mask * 1.0;
                mask(mask==0) = nan;
                likelyMasks(num2str(u_i)) = mask;
            end
        end
        
        %% Computes P(u | x, beta) \propto e^{beta * Q(x,u)}
        function pu = pugivenxbeta(obj, u, z, beta)
            % Return probability of control u given state x and model parameter theta.
            q_val = obj.q_fun(num2str(u));
            numerator = exp(beta * q_val.GetDataAtReal(z));
            denominator = 0;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_val = obj.q_fun(num2str(u_i));
                denominator = denominator + exp(beta * q_val.GetDataAtReal(z));
            end
            pu = numerator ./ denominator;
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
        
        %% Computes the Q-function for theta denoted by true_theta_idx 
        %   via value iteration. 
        function [v_grid, q_fun] = compute_q_fun(obj, gamma, eps)
            % Compute reward function
            % reward_info: gmin,gmax,gnums,g,obs_min,obs_max,obs_val
            r_fun = containers.Map;
            state_grid = obj.reward_info.g.xs;
            g_min = obj.reward_info.g.min';
            g_max = obj.reward_info.g.max';
            g_nums = obj.reward_info.g.N';
            
            % Create one mask which shows all states where obstacles are. 
%             penalty_in_obs_mask = 0 .* state_grid{1};
%             if obj.has_obs
%                 for oi = 1:length(obj.reward_info.obstacles)
%                     obs_info = obj.reward_info.obstacles{oi};
%                     obs_min = obs_info(1:2);
%                     obs_max = obs_info(1:2) + obs_info(3:4);
%                     obs_mask = (state_grid{1} >= obs_min(1)) & ...
%                         (state_grid{2} >= obs_min(2)) & ...
%                         (state_grid{1} <= obs_max(1)) & ...
%                         (state_grid{2} <= obs_max(2)); 
%                     % accumulate all obstacles in one mask. 
%                     penalty_in_obs_mask = penalty_in_obs_mask | obs_mask;
%                 end
%             end
            penalty_in_obs_mask = obj.reward_info.obstacles.data;
            
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                next_state = obj.physical_dynamics(state_grid, u_i);
                
                % -inf if moving outside grid
                penalty_outside_mask = (next_state{1} < g_min(1)) | ....
                                       (next_state{2} < g_min(2)) | ...
                                       (next_state{1} > g_max(1)) | ...
                                       (next_state{2} > g_max(2));
                penalty_outside = (penalty_outside_mask==0) .* 0.0 + ...
                                  (penalty_outside_mask==1) .* -obj.nearInf; 
                
                if obj.has_obs
%                     penalty_obstacle_mask = 0 .* state_grid{1};
%                     for oi = 1:length(obj.reward_info.obstacles)
%                         obs_info = obj.reward_info.obstacles{oi};
%                         obs_min = obs_info(1:2);
%                         obs_max = obs_info(1:2) + obs_info(3:4);
% 
%                         % -inf if moving into obstacle or inside of an obstacle rn.
%                         penalty_next_obs_mask = (next_state{1} >= obs_min(1)) & ...
%                                 (next_state{2} >= obs_min(2)) & ...
%                                 (next_state{1} <= obs_max(1)) & ...
%                                 (next_state{2} <= obs_max(2)); 
%                         penalty_obstacle_mask = penalty_next_obs_mask | penalty_in_obs_mask;  
%                     end
%                     penalty_obstacle = (penalty_obstacle_mask==0) .* 0.0 + ...
%                                         (penalty_obstacle_mask==1) .* -obj.nearInf;
                    penalty_obstacle_mask = obj.reward_info.obstacles.GetDataAtReal(next_state);
                    penalty_obstacle = (penalty_obstacle_mask==0) .* 0.0 + ...
                                    (penalty_obstacle_mask==1) .* -obj.nearInf;
                else
                    % if no obstacles, then no penalty for obstacle anywhere. 
                    penalty_obstacle = zeros(size(state_grid{1}));
                end
                
                %g = createGrid(g_min, g_max, g_nums)
                %surf(g.xs{1}, g.xs{2}, penalty_obstacle)
                
                % action costs
                if u_i(1) == 0 && u_i(2) == 0
                    % Stop
                    grid = Grid(g_min, g_max, g_nums);
                    grid.SetData(ones(size(next_state{1})) .* -obj.nearInf); 
                    grid.SetDataAtReal(obj.reward_info.theta, 0);
                    action_cost = grid.data;
                elseif u_i(1) ~= 0 && u_i(2) ~= 0
                    action_cost = ones(size(next_state{1})) .* -sqrt(2);
                else
                    action_cost = ones(size(next_state{1})) .* -1;
                end
                
                r_i = action_cost + penalty_obstacle + penalty_outside;
                r_fun(num2str(u_i)) = r_i;
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
                fprintf('    Max diff in value funs: %f, eps: %f\n', max_dev, eps);
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
                next_state = obj.physical_dynamics(state_grid, u_i);
                q_i = q_fun(num2str(u_i)).data;
                q_i(isInvalid) = 0;
                q = Grid(g_min, g_max, g_nums);
                
                if u_i(1)==0 && u_i(1)==0 % Ensure stop action is only equally likely if at goal.
                    q.SetData(ones(size(next_state{1})) .* -obj.nearInf); 
                    q.SetDataAtReal(obj.reward_info.theta, 0);
                end
                
                % -inf if moving outside grid or into obstacle
                penalty_outside_mask = (next_state{1} < g_min(1)) | ....
                                       (next_state{2} < g_min(2)) | ...
                                       (next_state{1} > g_max(1)) | ...
                                       (next_state{2} > g_max(2));
                penalty_outside = (penalty_outside_mask==0) .* 0.0 + ...
                                  (penalty_outside_mask==1) .* -obj.nearInf; 
                              
                if obj.has_obs
%                     penalty_obstacle_mask = 0 .* state_grid{1};
%                     for oi = 1:length(obj.reward_info.obstacles)
%                         obs_info = obj.reward_info.obstacles{oi};
%                         obs_min = obs_info(1:2);
%                         obs_max = obs_info(1:2) + obs_info(3:4);
% 
%                         % -inf if moving into obstacle or inside of an obstacle rn.
%                         penalty_next_obs_mask = (next_state{1} >= obs_min(1)) & ...
%                                 (next_state{2} >= obs_min(2)) & ...
%                                 (next_state{1} <= obs_max(1)) & ...
%                                 (next_state{2} <= obs_max(2)); 
%                         penalty_obstacle_mask = penalty_next_obs_mask | penalty_in_obs_mask;  
%                     end
%                     penalty_obstacle = (penalty_obstacle_mask==0) .* 0.0 + ...
%                                         (penalty_obstacle_mask==1) .* -obj.nearInf;
                    penalty_obstacle_mask = obj.reward_info.obstacles.GetDataAtReal(next_state);
                    penalty_obstacle = (penalty_obstacle_mask==0) .* 0.0 + ...
                                    (penalty_obstacle_mask==1) .* -obj.nearInf; 
                else
                    % if no obstacles, then no penalty for obstacle anywhere. 
                    penalty_obstacle = zeros(size(state_grid{1}));
                end
                
                q.SetData(q_i + penalty_obstacle + penalty_outside); 
                q_fun(num2str(u_i)) = q;
            end
        end
        
        %% Plots optimal policy for the inputted theta. 
        function plot_opt_policy(obj, theta_idx)
            q_fun = obj.q_fun;
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
%             if obj.has_obs
%                 for oi = 1:length(obj.reward_info.obstacles)
%                     obs_info = obj.reward_info.obstacles{oi};
%                     obs_min = obs_info(1:2);
%                         
%                     x_min = obs_min(1);
%                     y_min = obs_min(2);
%                     p_min = 1;
%                     l = [obs_info(3), ...
%                         obs_info(4), ...
%                         9];
%                     plotcube(l,[x_min y_min p_min], .5, [1 1 1]);
%                 end
%             end
            
            view(0,90);
            title(strcat("Optimal policy for theta=", num2str(theta_idx),". Color at state => opt control"));
        end
        
        %% Plots the probability of each action taken starting at init_z and under 
        %  the probability distribution conditioned on the trueTheta.
        function plot_pu_given_x(obj, init_z, thetas, trueThetaIdx)
            figure
            hold on
            % plot true goal.
            scatter(thetas{trueThetaIdx}(1), ...
                    thetas{trueThetaIdx}(2), ...
                    'r', 'filled');
            
            % plot obstacles.
            if obj.has_obs
                for oi = 1:length(obj.reward_info.obstacles)
                    obs_info = obj.reward_info.obstacles{oi};
                    rectangle('Position',obs_info)
                end
            end
            
            % plot init cond.
            scatter(init_z{1}, init_z{2}, 'b', 'filled');

            sum_pus = 0.0;
            controls_text = {'S', 'D', 'U', 'L', 'LD', 'LU', 'R', 'RD', 'RU'};
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                pu = obj.pugivenxtheta(u_i, init_z, obj.q_funs{trueThetaIdx});
                sum_pus = sum_pus + pu;
                fprintf(strcat('P(u = ',controls_text{i},') = ',num2str(pu),'\n'));
                znext = obj.dynamics(init_z,u_i);
                scatter(znext{1}, znext{2}, 'b');
                t = text(znext{1}-0.05, znext{2}+0.05, strcat('P(',controls_text{i},') : ',num2str(pu)));
                t.Color = [0,0,1];
                
                t = text(znext{1}-0.05, znext{2}-0.1, ...
                    strcat('b(g1|',controls_text{i},') : ',num2str(znext{3})));
                t.Color = [1,0,0];
            end
            fprintf("sum of pu's = %f\n", sum_pus);

            xlim([-4,4]);
            ylim([-4,4]);
        end
        
        %% Plots optimal policy for the inputted theta. 
        function plot_opt_policy_from_x0(obj, xinit, theta_idx)
            q_fun = obj.q_fun;
            all_q_vals = zeros([obj.reward_info.g.N', numel(obj.controls)]);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_i = q_fun(num2str(u_i)).data;
                all_q_vals(:,:,i) = q_i;
            end
            [opt_vals, opt_u_idxs] = max(all_q_vals, [], 3);
            
            % Colors for each theta
            if theta_idx == 1
                traj_color = [176, 0, 0]/255;
            else
                traj_color = [0, 106, 176]/255; 
            end
            scale = obj.gdisc(1);
            
            figure
            hold on
            uopts = Grid(obj.reward_info.g.min, ...
                        obj.reward_info.g.max, ...
                        obj.reward_info.g.N);
            uopts.SetData(opt_u_idxs);
            xcurr = xinit;
            opt_traj = [[xcurr{1};xcurr{2}]];
            for t=1:20
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
            scatter(obj.reward_info.theta{1}, ...
                obj.reward_info.theta{2}, ...
                'linewidth', 2, ...
                'marker', 'o', ...
                'markeredgecolor', g1Color, ...
                'markerfacecolor', g1Color);
            g1Txt = 'g1';
            t1 = text(obj.reward_info.theta{1}-0.05, ...
                        obj.reward_info.theta{2}-0.3, ...
                        0.55, g1Txt);
            t1.FontSize = 11;
            t1.Color = g1Color;
            
            % plot obstacle.
            bandw_cmap = [0,0,0;1,1,1]; %[1,1,1;0,0,0];
            colormap(bandw_cmap)
            ph = pcolor(obj.reward_info.obstacles.g{1}, ...
                        obj.reward_info.obstacles.g{2}, ...
                        obj.reward_info.obstacles.data);
            set(ph, 'EdgeColor', 'none');
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


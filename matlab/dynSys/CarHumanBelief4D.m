classdef CarHumanBelief4D < handle
    %CARHUMANBELIEF4D Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        thetas          % (cell) set of model parameters
        num_ctrls       % (int) number of discrete controls
        controls        % (arr) discretized controls
        z0              % (cell) initial joint state z = (x, y, b(theta_1))
        uThresh         % (float) threshold probability for sufficiently likely controls
        trueThetaIdx    % (int) true human intent parameter.
        v_funs          % (cell) Value functions (one entry per theta)
        q_funs          % (cell) Q-functions (one entry per theta)
        reward_info     % (struct) Information for reward function construction
        nearInf         % (float) Very large number which is < Inf 
        has_obs         % (bool) If this environment has obstacles or not.
        beta            % (float) beta \in [0, inf) which governs variance of likelihood model. 
        gnums           % (arr) number of discrete X, Y, Phi values.
        vel             % (float) fixed velocity of car
        dt              % (float) time discretization
        gdisc           % (arr) discretization resolution in x,y,phi,b(th=1)
        b_range         % (arr) range of belief values to clip to.

    end
    
    methods
        function obj = CarHumanBelief4D(z0, reward_info, trueThetaIdx, ...
                                        uThresh, gdisc, gnums, gamma, ...
                                        eps, beta, vel, dt, b_range)
            % CARHUMANBELIEF4D
            %   Represents joint dynamics of a 3D human-driven car 
            %   and a 2D discrete belief distribution. The belief is over
            %   two unknown goals, denoted by theta. The dynamics follow 
            %   an MDP grid structure where actions are UP/DOWN/LEFT/RIGHT
            %   and the four DIAGONALS and STOP. 
            %
            %   State:
            %       z = [px, py, phi, b(theta = th1)]
            %       where
            %           x = [px, py, phi] denotes the physical components
            %                             of the state
            %           b(theta = th1) denotes belief over first model
            %                           param.
            %   Dynamics:
            %       zdot = [v cos (phi), 
            %               v sin (phi),
            %               u 
            %               P(u | x, theta)b(theta = th1)/P(x,u)]
            %       where:
            %           P(u | x, theta) \propto e^{beta * Q(x,u,theta)}
            % 
            %       vel --  The arc length traveled, provided that the action
            %               involves changing position (x, y). `vel` is in units of
            %               gridsquares per timestep. It's recommended that `vel` is at
            %               least 1.
            %
            %       Q(x,u,theta) is computed via value iteration and the 
            %       reward function information packed into reward_info.
            %
            %   Controls:
            %       u \in {'STOP', 'FORWARD', 'FORWARD_CW1', 'FORWARD_CCW1'}
            %   
            obj.controls = obj.generate_controls_car();
            obj.num_ctrls = numel(obj.controls);
            obj.z0 = z0;
            obj.uThresh = uThresh;
            obj.trueThetaIdx = trueThetaIdx;
            obj.nearInf = 1000000000.0;
            obj.v_funs = cell(1,numel(reward_info.thetas));
            obj.q_funs = cell(1,numel(reward_info.thetas));
            obj.reward_info = reward_info;
            obj.beta = beta;
            obj.gnums = gnums;
            obj.vel = vel;
            obj.dt = dt;
            obj.gdisc = gdisc;
            obj.b_range = b_range;
            
            obj.has_obs = false;
            if isfield(obj.reward_info, 'obstacles')
                obj.has_obs = true;
            end
            
            % Pre-compute the Q-functions for each theta.
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
            znext_phys = obj.physical_dynamics(z, u);
            
            znext{1} = znext_phys{1};
            znext{2} = znext_phys{2};
            znext{3} = znext_phys{3};
            znext{4} = obj.belief_update(u,z);
        end
        
        %% Dyanmics of physical states.
        function znext = physical_dynamics(obj, z, u)
            % u \in {'STOP', 'FORWARD', 'FORWARD_CW1', 'FORWARD_CCW1'}
            
            znext = cell(3);
            
            if u == 1 % STOP
                znext{1} = z{1};
                znext{2} = z{2};
                znext{3} = z{3};
            elseif u == 2 % FORWARD
                znext{1} = z{1} + obj.dt * obj.vel * cos(z{3});
                znext{2} = z{2} + obj.dt * obj.vel * sin(z{3});
                znext{3} = z{3};
            else
                if u == 3 % FORWARD_CW1
                    ang_vel = (2*pi/obj.gnums(3))/obj.dt; 
                elseif u == 4 % FORWARD_CCW1
                    ang_vel = -(2*pi/obj.gnums(3))/obj.dt;
                else
                    error("Invalid control!")
                end
                znext{1} = z{1} + obj.vel/ang_vel * ...
                                (sin(z{3} + ang_vel*obj.dt) - sin(z{3}));
                znext{2} = z{2} - obj.vel/ang_vel * ...
                                (cos(z{3} + ang_vel*obj.dt) - cos(z{3}));
                znext{3} = mod(z{3} + ang_vel * obj.dt, 2*pi);
            end
        end
        
        %% Dynamics of the belief states (i.e. belief update)
        function bnext = belief_update(obj,u,z)
            % Calculate Bayesian posterior update.
            b0 = obj.pugivenxtheta(u, z, obj.q_funs{1}) .* z{4};
            b1 = obj.pugivenxtheta(u, z, obj.q_funs{2}) .* (1-z{4});
            normalizer = b0 + b1;
            bnext = b0 ./ normalizer;
            bnext = min(max(bnext, obj.b_range(1)), obj.b_range(2));
        end
        
        %% Mask over states for each control denoting if that control 
        %  is sufficiently likely at this state. 
        function likelyMasks = getLikelyMasks(obj, z)
            likelyMasks = containers.Map;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};

                % We want to choose controls such that:
                %   U = {u : P(u | x, theta = trueTheta) > delta}
                putheta = obj.pugivenxtheta(u_i, z, obj.q_funs{obj.trueThetaIdx});
                mask = (putheta > obj.uThresh);
                mask = mask * 1.0;
                mask(mask==0) = nan;
                likelyMasks(num2str(u_i)) = mask;
            end
        end
        
        %% Computes P(u | x, theta) \propto e^{beta * Q(x,u,theta)}
        function pu = pugivenxtheta(obj, u, z, q_fun)
            % Return probability of control u given state x and model parameter theta.
            q_val = q_fun(num2str(u));
            numerator = exp(obj.beta * q_val.GetDataAtReal(z));
            denominator = 0;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_val = q_fun(num2str(u_i));
                denominator = denominator + exp(obj.beta * q_val.GetDataAtReal(z));
            end
            pu = numerator ./ denominator;
        end
        
        %% Computes the Q-function for theta denoted by true_theta_idx 
        %   via value iteration. 
        function [v_grid, q_fun] = compute_q_fun(obj, true_theta_idx, gamma, eps)
            % Compute reward function
            % reward_info: gmin,gmax,gnums,g,obs_min,obs_max,obs_val
            r_fun = containers.Map;
            state_grid = obj.reward_info.g.xs;
            g_min = obj.reward_info.g.min';
            g_max = obj.reward_info.g.max';
            g_nums = obj.reward_info.g.N';
            
            % Create one mask which shows all states where obstacles are. 
            penalty_in_obs_mask = 0 .* state_grid{1};
            if obj.has_obs
                for oi = 1:length(obj.reward_info.obstacles)
                    obs_info = obj.reward_info.obstacles{oi};
                    obs_min = obs_info(1:2);
                    obs_max = obs_info(1:2) + obs_info(3:4);
                    obs_mask = (state_grid{1} >= obs_min(1)) & ...
                        (state_grid{2} >= obs_min(2)) & ...
                        (state_grid{1} <= obs_max(1)) & ...
                        (state_grid{2} <= obs_max(2)); 
                    % accumulate all obstacles in one mask. 
                    penalty_in_obs_mask = penalty_in_obs_mask | obs_mask;
                end
            end
            
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
                    penalty_obstacle_mask = 0 .* state_grid{1};
                    for oi = 1:length(obj.reward_info.obstacles)
                        obs_info = obj.reward_info.obstacles{oi};
                        obs_min = obs_info(1:2);
                        obs_max = obs_info(1:2) + obs_info(3:4);

                        % -inf if moving into obstacle or inside of an obstacle rn.
                        penalty_next_obs_mask = (next_state{1} >= obs_min(1)) & ...
                                (next_state{2} >= obs_min(2)) & ...
                                (next_state{1} <= obs_max(1)) & ...
                                (next_state{2} <= obs_max(2)); 
                        penalty_obstacle_mask = penalty_next_obs_mask | penalty_in_obs_mask;  
                    end
                    penalty_obstacle = (penalty_obstacle_mask==0) .* 0.0 + ...
                                        (penalty_obstacle_mask==1) .* -obj.nearInf;
                else
                    % if no obstacles, then no penalty for obstacle anywhere. 
                    penalty_obstacle = zeros(size(state_grid{1}));
                end
                
                % action costs
                if u_i == 1 % STOP ACTION
                    grid = Grid(g_min, g_max, g_nums);
                    grid.SetData(ones(size(next_state{1})) .* -obj.nearInf); 
                    grid.SetDataAtReal(obj.reward_info.thetas{true_theta_idx}, 0);
                    action_cost = grid.data;
                elseif u_i == 3 || u_i == 4 % FORWARD_CW or FORWARD_CCW 
                    action_cost = ones(size(next_state{1})) .* -sqrt(2); %-obj.vel*obj.dt;
                else % FORWARD
                    action_cost = ones(size(next_state{1})) .* -1; %-obj.vel*obj.dt;
                end
                
                r_i = action_cost + penalty_obstacle + penalty_outside;
                r_fun(num2str(u_i)) = r_i;
            end
            
            % Compute value function
            v_fun_prev = zeros(g_nums); 
            v_grid = Grid(g_min, g_max, g_nums);
            while true
                v_grid.SetData(v_fun_prev);
                possible_vals = zeros([g_nums, numel(obj.controls)]);
                for i=1:numel(obj.controls)
                    u_i = obj.controls{i};
                    next_state = obj.physical_dynamics(state_grid, u_i);
                    v_next = v_grid.GetDataAtReal(next_state);
                    possible_vals(:,:,:,i) = r_fun(num2str(u_i)) + (gamma .* v_next);
                end
                v_fun = max(possible_vals, [], 4);
                
                % \ell_\infty stopping condition
                v_delta = v_fun - v_fun_prev;
                max_dev = max(abs(v_delta),[],'all');
                fprintf('    max diff in values: %f, eps: %f\n', max_dev, eps);
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
            
            % ZERO ISSUE FIX
            % Make invalid states have Q-value of zero for all actions except
            % those that 1) are at the goal 2) moving into obstacle
            % 3) moving out of grid. 
            % States are invalid if \sum e^Q(x,u') == 0 (i.e no states give 
            % a non-zero action)
            isInvalid = (sum_exp == 0);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                next_state = obj.physical_dynamics(state_grid, u_i);
                q_i = q_fun(num2str(u_i)).data;
                q_i(isInvalid) = 0;
                q = Grid(g_min, g_max, g_nums);
                
                if i==1 % Ensure stop action is only equally likely if at goal.
                    q.SetData(ones(size(next_state{1})) .* -obj.nearInf); 
                    q.SetDataAtReal(obj.reward_info.thetas{true_theta_idx}, 0);
                end
                
                % -inf if moving outside grid or into obstacle
                penalty_outside_mask = (next_state{1} < g_min(1)) | ....
                                       (next_state{2} < g_min(2)) | ...
                                       (next_state{1} > g_max(1)) | ...
                                       (next_state{2} > g_max(2));
                penalty_outside = (penalty_outside_mask==0) .* 0.0 + ...
                                  (penalty_outside_mask==1) .* -obj.nearInf; 
                              
                if obj.has_obs
                    penalty_obstacle_mask = 0 .* state_grid{1};
                    for oi = 1:length(obj.reward_info.obstacles)
                        obs_info = obj.reward_info.obstacles{oi};
                        obs_min = obs_info(1:2);
                        obs_max = obs_info(1:2) + obs_info(3:4);

                        % -inf if moving into obstacle or inside of an obstacle rn.
                        penalty_next_obs_mask = (next_state{1} >= obs_min(1)) & ...
                                (next_state{2} >= obs_min(2)) & ...
                                (next_state{1} <= obs_max(1)) & ...
                                (next_state{2} <= obs_max(2)); 
                        penalty_obstacle_mask = penalty_next_obs_mask | penalty_in_obs_mask;  
                    end
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
            q_fun = obj.q_funs{theta_idx};
            all_q_vals = zeros([obj.reward_info.g.N', numel(obj.controls)]);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_i = q_fun(num2str(u_i)).data;
                all_q_vals(:,:,:,i) = q_i;
            end
            [opt_vals, opt_u_idxs] = max(all_q_vals, [], 4);
            
            figure
            hold on
            % plot the optimal action at each state.
            
            
            % TODO: GOTTA PROJECT INTO 2D THE G.XS
            g2D = createGrid(obj.reward_info.g.min(1:2), ...
                            obj.reward_info.g.max(1:2), ...
                            obj.reward_info.g.N(1:2));
            surf(g2D.xs{1}, g2D.xs{2}, opt_u_idxs(:,:,1));
            figure 
            surf(obj.reward_info.g.xs{1}, obj.reward_info.g.xs{2}, opt_u_idxs(:,:,2));
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
        
        %% Gets x-y-phi-belief trajectory of optimal path.
        function z_traj = get_opt_policy_joint_traj(obj, grid, ...
                zinit, theta_idx, target_b)
            q_fun = obj.q_funs{theta_idx};
            all_q_vals = zeros([obj.reward_info.g.N', numel(obj.controls)]);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_i = q_fun(num2str(u_i)).data;
                all_q_vals(:,:,:,i) = q_i;
            end
            [opt_vals, opt_u_idxs] = max(all_q_vals, [], 4);
            uopts = Grid(obj.reward_info.g.min, ...
                        obj.reward_info.g.max, ...
                        obj.reward_info.g.N);
            uopts.SetData(opt_u_idxs);
            
            gridded_zinit = grid.RealToCoords(zinit);
            zcurr = gridded_zinit;
            z_traj = {};
            z_traj{end+1} = zcurr;
            while true
                % if we reach the target belief. 
                if (zcurr{4} >= target_b && theta_idx == 1) || ...
                    (zcurr{4} <= target_b && theta_idx == 2)
                    break;
                end
                xcurr = zcurr(1:3);
                % get optimal control at this state
                uopt = uopts.GetDataAtReal(xcurr);
                % propagate dynamics
                znext = obj.dynamics(zcurr,uopt);
                gridded_znext = grid.RealToCoords(znext); 
                z_traj{end+1} = gridded_znext;
                zcurr = gridded_znext;
            end
        end
        
        %% Plots optimal policy for the inputted theta. 
        function plot_opt_policy_from_x0(obj, xinit, theta_idx)
            q_fun = obj.q_funs{theta_idx};
            all_q_vals = zeros([obj.reward_info.g.N', numel(obj.controls)]);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_i = q_fun(num2str(u_i)).data;
                all_q_vals(:,:,:,i) = q_i;
            end
            [opt_vals, opt_u_idxs] = max(all_q_vals, [], 4);
            
            % 
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
            for t=1:20
                % plot the current state.
                quiver(xcurr{1}, ...
                        xcurr{2}, ...
                        cos(xcurr{3}), ...
                        sin(xcurr{3}), scale, ...
                        'Color', traj_color, ...
                        'linewidth', 2, ...
                        'marker', 'o', ...
                        'markeredgecolor', traj_color, ...
                        'markerfacecolor', traj_color);
                % get optimal control at this state
                uopt = uopts.GetDataAtReal(xcurr);
                xcurr = obj.physical_dynamics(xcurr, uopt);
            end 
            
            % Goal colors.
            g1Color = 'r';
            g2Color = [38., 138., 240.]/255.;
    
            % g1
            quiver(obj.reward_info.thetas{1}{1}, ...
                obj.reward_info.thetas{1}{2}, ...
                cos(obj.reward_info.thetas{1}{3}), ...
                sin(obj.reward_info.thetas{1}{3}), scale, ...
                'Color', g1Color, ...
                'linewidth', 2, ...
                'marker', 'o', ...
                'markeredgecolor', g1Color, ...
                'markerfacecolor', g1Color);
            g1Txt = 'g1';
            t1 = text(obj.reward_info.thetas{1}{1}-0.05, ...
                        obj.reward_info.thetas{1}{2}-0.3, ...
                        0.55, g1Txt);
            t1.FontSize = 11;
            t1.Color = g1Color;
            
            % g2
            quiver(obj.reward_info.thetas{2}{1}, ...
                obj.reward_info.thetas{2}{2}, ...
                cos(obj.reward_info.thetas{2}{3}), ...
                sin(obj.reward_info.thetas{2}{3}), scale, ...
                'Color', g2Color, ...
                'linewidth', 2, ...
                'marker', 'o', ...
                'markeredgecolor', g2Color, ...
                'markerfacecolor', g2Color);
            g2Txt = 'g2';
            t2 = text(obj.reward_info.thetas{2}{1}-0.2, ...
                        obj.reward_info.thetas{2}{2}-0.3, ...
                        0.55, g2Txt);
            t2.FontSize = 11;
            t2.Color = g2Color;
            
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
            xlim([obj.reward_info.g.min(1), obj.reward_info.g.max(1)]);
            ylim([obj.reward_info.g.min(2), obj.reward_info.g.max(2)]);
            set(gcf, 'color', 'w');
            set(gcf, 'position', [0,0,800,800]);
            box on;
            view(0,90);
            title(strcat("Optimal policy for theta=", num2str(theta_idx),"."));
        end
        
        %% Generates the discrete controls for the MDP-CAR human. 
        function controls = generate_controls_car(obj)
            % u \in {'STOP', 'FORWARD', 'FORWARD_CW1', 'FORWARD_CCW1'}
            controls = {1,2,3,4};
        end
        
        %% Given the previous and the next physical states, 
        % inverts the dynamics and returns the closest control that was
        % taken. 
        function uidx = inv_dyn(obj, xnext, xprev)
            
            if all(abs(xnext - xprev) < 1e-04) 
                % if no change between two states, action is ABSORB
                uidx = 1; % STOP ACTION
                return
            end
            
            if all(abs(xnext(3) - xprev(3)) < 1e-04) 
                % if no change in the angle, action is FORWARD
                uidx = 2;
                return;
            else
                % determine which of the binned controls is most like the applied one
                u = (xnext(3) - xprev(3))/obj.dt;
                fwdccw1 = (2*pi/obj.gnums(3))/obj.dt;
                uarr = ones(1,2)*u;
                binned_u = [fwdccw1, -fwdccw1];
                action_array = [3,4]; % 'FORWARD_CW1', 'FORWARD_CCW1' 
                
                % look at difference between the applied control and all binned controls
                diff = abs(uarr - binned_u);
                % get the index of min difference
                [minv, min_idx] = min(diff);
                
                % return the binned action that is most similar to the applied control
                uidx = action_array(min_idx);
                return; 
            end 
        end
    end
end


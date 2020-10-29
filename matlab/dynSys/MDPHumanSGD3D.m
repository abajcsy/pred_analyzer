classdef MDPHumanSGD3D < handle

    properties
        num_ctrls       % (int) number of discrete controls
        controls        % (arr) discretized controls
        z0              % (cell) initial joint state z = (x, y, theta)
        uThresh         % (float) threshold probability for sufficiently likely controls
        trueTheta       % (float) true human reward weight.
        v_funs          % (cell) Value functions (one entry per theta)
        q_funs          % (cell) Q-functions (one entry per theta)
        validMasks      % (cell) masks that check if action goes into obstacle or out of grid (for each control)
        reward_info     % (struct) Information for reward function construction
        nearInf         % (float) Very large number which is < Inf 
        has_obs         % (bool) If this environment has obstacles or not.
        gdisc           % (arr) discretization resolution in x,y,theta
        alpha           % (float) step size for SGD update
        w1              % (float) weight on distance-to-goal (pre-specified)
        
        goal_feature    % (Map) phi(x,u) for distance-to-goal feature
        obs_feature     % (Map) phi(x,u) for distance-to-obstacles feature
        grid3d          % (struct) Grid structure for joint state space.
        interp_Qfuns    % (matrix) nx * ny * nu * ntheta matrix of Q-values
        accuracy        % (string) accuracy of numeric derivatives
        
        obs_padding  % (float) scales up or down the obstacle feature
        beta         % (float) scales the variance of the distribution. 
    end
    
    methods
        function obj = MDPHumanSGD3D(z0, reward_info, trueTheta, ...
                                        uThresh, gdisc, gamma, ...
                                        eps, alpha, w1, grid3d, ...
                                        accuracy, obs_padding, beta)
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
            %               theta + alpha * \nabla_theta F(x, u, theta)
            %       where:
            %           F(x, u, theta) := 
            %               Q(x, u, theta) - E_{u ~ P(u | x, theta)}[Q(x,u,theta)]
            %           and 
            %           \nabla_theta  F(x, u, theta_n-1)
            %           is the gradient of F wrt. theta for a given (x,u) pair. 
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
            obj.nearInf = 100.0; % NOTE: we cannot make this true near inf 
                                 % because numerically approximating the 
                                 % becomes infeasible as we approach Inf 
                                 % and the function becomes less and
                                 % less differentiable  
            obj.v_funs = cell(1,numel(reward_info.thetas));
            obj.q_funs = cell(1,numel(reward_info.thetas));
            obj.reward_info = reward_info;
            obj.gdisc = gdisc;
            obj.alpha = alpha;
            obj.w1 = w1;
            obj.grid3d = grid3d;
            obj.accuracy = accuracy;
            obj.obs_padding = obs_padding;
            obj.beta = beta;
            
            obj.has_obs = false;
            if isfield(obj.reward_info, 'obstacles')
                obj.has_obs = true;
            end
            
            % Pre-compute the reward features. 
            %       phi(x,u) = [dist_to_goal(x,u), dist_to_obs(x,u)]    
            obj.goal_feature = obj.compute_goal_feature(reward_info);
            obj.obs_feature = obj.compute_obs_feature(reward_info);
            
            obj.validMasks = cell(1, obj.num_ctrls);
            for i=1:obj.num_ctrls
                obj.validMasks{i} = obj.checkValidAction(obj.reward_info.g.xs,obj.controls{i});
            end
            
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
            
            fprintf('Interpolating Q-functions for all thetas in state space ...\n');
            % Make a matrix of size:
            %   (num_xs, num_ys, num_us, num_precomp_thetas) 
            all_Qfuns = zeros([grid3d.N(1), ...
                                grid3d.N(2), ...
                                length(obj.controls), ...
                                length(obj.reward_info.thetas)]);
            for ti = 1:length(obj.reward_info.thetas)
                for ui = 1:length(obj.controls)
                    u_curr = obj.controls{ui};
                    all_Qfuns(:,:,ui,ti) = ...
                        obj.q_funs{ti}(num2str(u_curr)).data;
                end
            end
            
            % Interpolate the Q-functions for all theta values. 
            precomp_thetas = cell2mat(obj.reward_info.thetas);
            [xs,ys,us,ts] = ndgrid(grid3d.min(1):grid3d.dx(1):grid3d.max(1),...
                                   grid3d.min(2):grid3d.dx(2):grid3d.max(2),...
                                   1:1:length(obj.controls),...
                                   precomp_thetas);
            [interp_xs,interp_ys,interp_us,interp_ts] = ...
                                    ndgrid(grid3d.min(1):grid3d.dx(1):grid3d.max(1),...
                                       grid3d.min(2):grid3d.dx(2):grid3d.max(2),...
                                       1:1:length(obj.controls),...
                                       grid3d.min(3):grid3d.dx(3):grid3d.max(3));  
                                   
            obj.interp_Qfuns = interpn(xs,ys,us,ts,all_Qfuns,...
                                    interp_xs,interp_ys,interp_us,interp_ts);
                                
            % ===== DEBUGGING! ===== %
%             uidx = 7; % 7 = right action, 4 = left action
%             z1 = {-2, -1, -1};
%             z2 = {-2, -1, -0.5};
%             z3 = {-2, -1, 0};
%             z4 = {-2, -1, 0.5};
%             z5 = {-2, -1, 1};
%             qth1 = eval_u(obj.grid3d, squeeze(obj.interp_Qfuns(:,:,uidx,:)), cell2mat(z1));
%             qth2 = eval_u(obj.grid3d, squeeze(obj.interp_Qfuns(:,:,uidx,:)), cell2mat(z2));
%             qth3 = eval_u(obj.grid3d, squeeze(obj.interp_Qfuns(:,:,uidx,:)), cell2mat(z3));
%             qth4 = eval_u(obj.grid3d, squeeze(obj.interp_Qfuns(:,:,uidx,:)), cell2mat(z4));
%             qth5 = eval_u(obj.grid3d, squeeze(obj.interp_Qfuns(:,:,uidx,:)), cell2mat(z5));
%             plot([0,0.25,0.5,0.75,1], [qth1, qth2, qth3, qth4, qth5]);
%             bla = 1;

            % ======= PLOT THE Q-func @ x for all u and theta ====== %
%             debugging_qs = zeros([length(obj.controls), length(0:obj.gdisc(3):1)]);
%             for uidx = 1:length(obj.controls)
%                 thidx = 1;
%                 for th = 0:obj.gdisc(3):1
%                     z = {0,0,th}; %{-1,2,th}; %{-2, -1, th};
%                     qth = eval_u(obj.grid3d, squeeze(obj.interp_Qfuns(:,:,uidx,:)), cell2mat(z));
%                     debugging_qs(uidx, thidx) = qth;
%                     thidx = thidx + 1;
%                 end
%             end
% 
%             [U,T] = meshgrid(0:obj.gdisc(3):1,1:1:length(obj.controls));
%             surf(U,T,debugging_qs);
%             yticklabels({'S', 'D', 'U', 'L', 'LD', 'LU', 'R', 'RD', 'RU'});
%             xticks([0,0.5,1]);
%             yticks([1,2,3,4,5,6,7,8,9]);
%             xlabel('theta');
%             ylabel('action');
%             zlabel('Q(x=-2, y=-1, u, theta)');
%             grid on;
%             set(gcf, 'color', 'w')
%             box on;
            % ===== DEBUGGING! ===== %
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
        %   theta_n = theta_n-1 + \alpha * \nabla_theta F(x,u,theta_n-1)
        %
        %   where F(x,u,theta_n-1) = 
        %       Q(x, u, theta_n-1) - E_{u ~ P(u | x, theta_n-1)}[Q(x,u,theta_n-1)]
        %
        %  Uses a finite-difference approximation to \nabla_theta F
        function [theta_n, dF_final] = sgd(obj,u,z)
           
            % Find index of this control in set of controls.
            mat_ctrls = reshape(cell2mat(obj.controls), [2,length(obj.controls)]);
            diff = abs(mat_ctrls - u');
            uidx = find(sum(diff,1) < 0.0001);
            
            if isempty(uidx)
                error("Unable to find index of given control in SGD!");
            end
            
            % TODO: most of these lines can be in a precomputation step in
            % the constructor to save time. 
            
            % Compute: E_{u ~ P(u | x, theta_n-1)}[Q(x,u,theta_n-1)])
            expected_Q = zeros(size(obj.grid3d.xs{1}));
            for ui=1:length(obj.controls)
                % Get P(u | <all x>, <all theta>)
                pu_given_x_theta = obj.pugivenxtheta(u, obj.grid3d.xs);
                expected_Q = expected_Q + squeeze(obj.interp_Qfuns(:,:,uidx,:)) .* pu_given_x_theta;
            end
            
            % Compute: (Q(x, u, theta_n-1) -  E_{u ~ P(u | x, theta_n-1)}[Q(x,u,theta_n-1)])
            grad_func = squeeze(obj.interp_Qfuns(:,:,uidx,:)) - expected_Q;
            
            % Approximate the gradient:
            %   \nabla_theta (Q(x, u, theta_n-1) -  E_{u ~ P(u | x, theta_n-1)}[Q(x,u,theta_n-1)])
            low = [obj.grid3d.min(1), obj.grid3d.min(2), obj.grid3d.min(3)];
            up = [obj.grid3d.max(1), obj.grid3d.max(2), obj.grid3d.max(3)];
            N = [obj.grid3d.N(1), obj.grid3d.N(2), obj.grid3d.N(3)];
            deriv_grid = createGrid(low,up,N);
            deriv_dim = 3;          % taking spatial gradient of the theta dim
            generateAll = 0;        % ignore all possible second order upwind approximations.

            % Choose fidelty of numerical approximation to gradient. 
            if strcmp(obj.accuracy, 'low')
                [derivF_L, derivF_R] = ...
                   upwindFirstFirst(deriv_grid, grad_func, deriv_dim, generateAll);
            elseif strcmp(obj.accuracy, 'medium')
                [derivF_L, derivF_R ] = ...
                     upwindFirstENO2(deriv_grid, grad_func, deriv_dim, generateAll);
            elseif strcmp(obj.accuracy, 'high')
                [derivF_L, derivF_R ] = ...
                     upwindFirstENO3(deriv_grid, grad_func, deriv_dim, generateAll);
            else
                error("Invalid accuracy type!\n");
            end
            
            % Compute the central derivative
            dF = 0.5 .* (derivF_L + derivF_R);

            % Numerically approximate: \nabla_theta Q(x,u,theta_n-1)
            %dF = squeeze(derivF_central(:,:,uidx,:)); % squeeze to remove dim of size = 1

            % Extract all theta values.
            theta = z{3};
            
            % HACK?! need to think of a better way to do this
            if size(theta) == 1
                % We are querying for SGD at *single* (x,y,theta) point
                dF_final = eval_u(obj.grid3d, dF, cell2mat(z));
            else
                % We are querying for SGD at *grid* of (x,y,theta) points
                dF_final = dF;
            end
            
            % Perform SGD update!
            theta_n = theta + obj.alpha .* dF_final;
            
            % Clip next theta to be between max and min 
            % acceptable weight values.
            theta_n = min(max(theta_n, obj.grid3d.min(3)), obj.grid3d.max(3));
        end
        
        %% Computes P(u | x, theta) \propto e^{Q(x,u,theta)}
        function pu = pugivenxtheta(obj, u, z)
            g_min = obj.reward_info.g.min';
            g_max = obj.reward_info.g.max';
            g_nums = obj.reward_info.g.N';
            
            grid = Grid(g_min, g_max, g_nums);
            
            % Find index of this control in set of controls.
            mat_ctrls = reshape(cell2mat(obj.controls), [2,length(obj.controls)]);
            diff = abs(mat_ctrls - u');
            uidx = find(sum(diff,1) < 0.0001);
            
            % Get Q(<all x>, <all y>, u, <all theta>)
            q_val = squeeze(obj.interp_Qfuns(:,:,uidx,:));
            %qvals = eval_u(obj.grid3d, qfun, cell2mat(z));
            
            % Return probability of control u given state x and model parameter theta.
            validMask = obj.validMasks{uidx};
            numerator = exp(obj.beta .* q_val) .* validMask;
            denominator = 0;
            for i=1:obj.num_ctrls
                validMask = obj.validMasks{i};
                q_val = squeeze(obj.interp_Qfuns(:,:,i,:));
                denominator = denominator + (exp(obj.beta .* q_val) .* validMask);
            end
            
            % Returns P(u | <all x>, <all theta>)
            pu = numerator ./ denominator;
            
            % For states with zero exp, make them uniformally distributed
            % across action
            isInvalid = (denominator == 0);
            pu(isInvalid) = 1/obj.num_ctrls;
        end
        
        %% Generates the discrete controls for the MDP human. 
        function controls = generate_controls_mdp(obj, gdisc)
            nc = 9;
            controls = cell(1,nc);
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
            %obs_padded = -1 .* imdilate(-1 .* reward_info.obstacles.data, eye(obj.obs_padding));
            
            % convert the signed distance into a "bump distance"
            % where phi(x,u) = 0 if outside obstacle
            % and   phi(x,u) = signedDist inside obstacle
            obs_padded = reward_info.featurized_obstacles.data;
            obs_padded(reward_info.featurized_obstacles.data >= 0) = 0;
            obs_feature = obs_padded;
            
        end
        
        %% Checks if next state is in an obstacle or outside comp grid. 
        function validAction = checkValidAction(obj,z,u)
            next_state = obj.physical_dynamics(z, u);
            g_min = obj.reward_info.g.min';
            g_max = obj.reward_info.g.max';
            g_nums = obj.reward_info.g.N';
            
            penalty_outside_mask = (next_state{1} < g_min(1)) | ....
                                   (next_state{2} < g_min(2)) | ...
                                   (next_state{1} > g_max(1)) | ...
                                   (next_state{2} > g_max(2));

            if obj.has_obs
                penalty_obstacle_mask = obj.reward_info.obstacles.GetDataAtReal(next_state);
            else
                % if no obstacles, then no penalty for obstacle anywhere. 
                penalty_obstacle_mask = zeros(size(z{1}));
            end
            
            validAction = ~(penalty_outside_mask == 1 | penalty_obstacle_mask  == 1);
%             validAction = ~(penalty_outside_mask==1);
            validAction = validAction .* 1.0;
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
%                 phi_g(isnan(phi_g)) = -obj.nearInf;
%                 phi_o(isnan(phi_o)) = -obj.nearInf;

                % -inf if moving outside grid
                penalty_outside_mask = (next_state{1} < g_min(1)) | ....
                                       (next_state{2} < g_min(2)) | ...
                                       (next_state{1} > g_max(1)) | ...
                                       (next_state{2} > g_max(2));
                
                if obj.has_obs
                    penalty_obstacle_mask = obj.reward_info.obstacles.GetDataAtReal(next_state);
                else
                    % if no obstacles, then no penalty for obstacle anywhere. 
                    penalty_obstacle_mask = zeros(size(state_grid{1}));
                end
                
                phi_g = reshape(phi_g, g_nums(1), g_nums(2));
                phi_o = reshape(phi_o, g_nums(1), g_nums(2));
                
                % Compute: r(x,u) = w^T * phi(x,u)
                %   where the reward is -inf if the action takes the
                %   agent outside of the grid and = 0 else.
                %r_i = obj.w1 .* phi_g + w2 .* phi_o;
                r_i = (1-w2) .* phi_g + w2 .* phi_o;
                r_i(penalty_outside_mask==1) = -1 .* obj.nearInf;
                
                if u_i(1)==0 && u_i(2)==0 % Ensure stop action is only equally likely if at goal.
                    grid = Grid(g_min, g_max, g_nums);
                    grid.SetData(ones(size(next_state{1})) .* -obj.nearInf);
                    goal = {obj.reward_info.goal(1), obj.reward_info.goal(2)};
                    grid.SetDataAtReal(goal, 0);
                    r_i = grid.data;
                end
                
                r_fun(num2str(u_i)) = r_i;
                
                % === plot reward landscpe === %
%                 surf(obj.reward_info.g.xs{1},obj.reward_info.g.xs{2}, r_i)
%                 colorbar
%                 bla=1;
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
                next_state = obj.physical_dynamics(state_grid, u_i);
                q_i = q_fun(num2str(u_i)).data;
                q_i(isInvalid) = 0;
                q = Grid(g_min, g_max, g_nums);
                
%                 if u_i(1)==0 && u_i(2)==0 % Ensure stop action is only equally likely if at goal.
%                     q.SetData(ones(size(next_state{1})) .* -obj.nearInf); 
%                     q.SetDataAtReal(obj.reward_info.thetas{true_theta_idx}, 0);
%                 end

                % -inf if moving outside grid or into obstacle
                % -inf if moving outside grid
                penalty_outside_mask = (next_state{1} < g_min(1)) | ....
                                       (next_state{2} < g_min(2)) | ...
                                       (next_state{1} > g_max(1)) | ...
                                       (next_state{2} > g_max(2));
                
                if obj.has_obs
                    penalty_obstacle_mask = obj.reward_info.obstacles.GetDataAtReal(next_state);
                else
                    % if no obstacles, then no penalty for obstacle anywhere. 
                    penalty_obstacle_mask = zeros(size(state_grid{1}));
                end
                                   
                q_i(penalty_outside_mask==1) = -obj.nearInf; 
                q.SetData(q_i);
                
                if u_i(1)==0 && u_i(2)==0 % Ensure stop action is only equally likely if at goal.
                    goal = {obj.reward_info.goal(1), obj.reward_info.goal(2)};
                    val_at_goal = q.GetDataAtReal(goal);
                    q = Grid(g_min, g_max, g_nums);
                    q.SetData(ones(size(next_state{1})) .* -obj.nearInf);
                    q.SetDataAtReal(goal, val_at_goal);
                end
                
                q_fun(num2str(u_i)) = q;
            end
        end
        
        %% Mask over states for each control denoting if that control 
        %  is sufficiently likely at this state. 
        function likelyMasks = getLikelyMasks(obj, z)
            likelyMasks = containers.Map;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};

                % We want to choose controls such that:
                %   U = {u : P(u | x, theta = trueTheta) > delta}
                putheta = obj.pugivenxtheta(u_i, z);
                mask = (putheta > obj.uThresh);
                mask = mask * 1.0;
                mask(mask==0) = nan;
                likelyMasks(num2str(u_i)) = mask;
            end
        end

        %% Plots optimal policy for the inputted theta. 
        function plot_opt_policy(obj, theta_idx)
            %q_fun = obj.q_funs{theta_idx};
            q_fun = obj.interp_Qfuns(:,:,:,theta_idx);
            all_q_vals = zeros([obj.reward_info.g.N', numel(obj.controls)]);
                        
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                
                % Find index of this control in set of controls.
                mat_ctrls = reshape(cell2mat(obj.controls), [2,length(obj.controls)]);
                diff = abs(mat_ctrls - u_i');
                uidx = find(sum(diff,1) < 0.0001);

                q_i = q_fun(:,:,uidx,:); %q_fun(num2str(u_i)).data;
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
                % Visualize black and white 
%                 bandw_cmap = [1,1,1;0,0,0];
%                 colormap(bandw_cmap)
                ph = pcolor(obj.reward_info.featurized_obstacles.g{1}, ...
                        obj.reward_info.featurized_obstacles.g{2}, ...
                        obj.reward_info.featurized_obstacles.data);
                set(ph, 'EdgeColor', 'none');
            end
            
            view(0,90);
            title(strcat("Optimal policy for theta=", num2str(theta_idx),". Color at state => opt control"));
        end
        
        %% Gets the optimal trajectory for a specific theta index.
        function opt_traj = get_opt_traj(obj, z, theta)
%             q_fun = obj.q_funs{theta_idx};
%             all_q_vals = zeros([obj.reward_info.g.N', numel(obj.controls)]);
            z_phys = obj.reward_info.g.xs;
            g = obj.grid3d;
            
            pu_vals = zeros([g.shape, numel(obj.controls)]);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                pu_grid = obj.pugivenxtheta(u_i, z_phys);
                pu_vals(:,:,:,i) = pu_grid;
            end
            
            xcurr = z;
            opt_traj = [[xcurr{1};xcurr{2}]];
            d_to_goal = norm([xcurr{1}; xcurr{2}] - obj.reward_info.goal);
            iter = 1;
            
            validMaskGrid = Grid([g.axis(1), g.axis(3)], ...
                                [g.axis(2), g.axis(4)], ...
                                [g.shape(1), g.shape(2)]);
            
            while d_to_goal >= eps 
                if iter > 80
                    break;
                end
                % get optimal control at this state
                pu_single_vals = zeros(1,numel(obj.controls));
                for i=1:obj.num_ctrls
                    zcurr = {xcurr{1},xcurr{2},theta};
                    validMask = obj.validMasks{i};
                    validMaskGrid.SetData(validMask);
                    isValid = validMaskGrid.GetDataAtReal(xcurr);
                    pu_single = eval_u(g, pu_vals(:,:,:,i), cell2mat(zcurr), 'nearest');
                    pu_single_vals(i) = pu_single;
                end
                [uopt_val, uopt_idx] = max(pu_single_vals);
                
                xcurr = obj.physical_dynamics(xcurr, obj.controls{uopt_idx});
                opt_traj(:,end+1) = [xcurr{1};xcurr{2}];
                d_to_goal = norm([xcurr{1}; xcurr{2}] - obj.reward_info.goal);
                iter = iter + 1;
            end 
        end
        
        %% Plots optimal policy for the inputted theta. 
        function plot_opt_policy_from_x0(obj, xinit, theta)
            opt_traj = obj.get_opt_traj(xinit, theta);
            
            % Colors for each theta
            num_thetas = length(obj.reward_info.thetas);
            scale = obj.gdisc(1);
            
            theta_idx = 1;
            
            traj_start_color = [176, 0, 0]/255;
            traj_end_color = [0, 106, 176]/255; 
            rs = linspace(traj_start_color(1), traj_end_color(1), num_thetas);
            gs = linspace(traj_start_color(2), traj_end_color(2), num_thetas);
            bs = linspace(traj_start_color(3), traj_end_color(3), num_thetas);
            
            traj_color = [rs(theta_idx), gs(theta_idx), bs(theta_idx)];
            
            figure
            hold on
            
            % Plot lines connecting each of the state dots. 
            plot(opt_traj(1,:), opt_traj(2,:), 'linewidth', 2);
            
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
                % Visualize black and white 
%                 bandw_cmap = [1,1,1;0,0,0];
%                 colormap(bandw_cmap)
                ph = pcolor(obj.reward_info.featurized_obstacles.g{1}, ...
                        obj.reward_info.featurized_obstacles.g{2}, ...
                        obj.reward_info.featurized_obstacles.data);
                set(ph, 'EdgeColor', 'none');
            end
            set(gcf,'color','w');
            xlim([obj.reward_info.g.min(1), obj.reward_info.g.max(1)]);
            ylim([obj.reward_info.g.min(2), obj.reward_info.g.max(2)]);
            set(gcf, 'Position', [0,0,800,800])
            box on
            grid off
            view(0,90);
            title(strcat("Optimal policy for theta=", num2str(theta),"."));
        end
        
    end
end


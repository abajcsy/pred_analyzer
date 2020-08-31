classdef MDPHumanBelief2D_R < handle
    % Human belief 1D class
    
    properties
        thetas % set of model parameters
        num_ctrls % number of discrete controls
        controls % discretized controls
        z0 % initial joint state z = (x, y, b(theta_1))
        uThresh % threshold probability for sufficiently likely controls
        trueThetaIdx % true human intent parameter.
        v_funs % Value functions
        q_funs % Q-functions
        reward_info % Information for reward function construction
        
        % Note: Is it safe to assume b scalar or can we have len(theta)>2
        % (which would mean to have b of dimension len(theta)-1)?
    end
    
    methods
        function obj = MDPHumanBelief2D_R(z0, reward_info, trueThetaIdx, ...
                uThresh, gdisc, gamma, eps)
            % Construct an instance of Grid.
            obj.controls = obj.generate_controls_mdp(gdisc);
            obj.num_ctrls = numel(obj.controls);
            obj.z0 = z0;
            obj.uThresh = uThresh;
            obj.trueThetaIdx = trueThetaIdx;
            obj.v_funs = cell(1,numel(reward_info.thetas));
            obj.q_funs = cell(1,numel(reward_info.thetas));
            obj.reward_info = reward_info;
            for th_idx=1:numel(reward_info.thetas)
                [v_f, q_f] = obj.compute_q_fun(th_idx, gamma, eps);
                obj.v_funs{th_idx} = v_f;
                obj.q_funs{th_idx} = q_f;
            end
        end
        
        function znext = dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(size(z));
            znext{1} = z{1} + u(1);
            znext{2} = z{2} + u(2);
            znext{3} = obj.belief_update(u,z);
        end
        
        function znext = physical_dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(2);
            znext{1} = z{1} + u(1);
            znext{2} = z{2} + u(2);
        end
        
        function bnext = belief_update(obj,u,z)
            % Calculate Bayesian posterior update.
            b0 = obj.pugivenxtheta(u, z, obj.q_funs{1}) .* z{3};
            b1 = obj.pugivenxtheta(u, z, obj.q_funs{2}) .* (1-z{3});
            normalizer = b0 + b1;
            bnext = b0 ./ normalizer;
        end
        
        function likelyMasks = getLikelyMasks(obj, z)
            likelyMasks = containers.Map;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};

                % We want to choose controls such that:
                %   U = {u : P(u | x, theta = trueTheta) >= delta}
                putheta = obj.pugivenxtheta(u_i, z, obj.q_funs{obj.trueThetaIdx});
                mask = (putheta >= obj.uThresh);
                mask = mask * 1.0;
                mask(mask==0) = nan;
                likelyMasks(num2str(u_i)) = mask;
            end
        end
        
        function pu = pugivenxtheta(obj,u,z,q_fun)
            % Return probability of control u given state x and model parameter theta.
            q_val = q_fun(num2str(u));
            numerator = exp(q_val.GetDataAtReal(z));
            denominator = 0;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_val = q_fun(num2str(u_i));
                denominator = denominator + exp(q_val.GetDataAtReal(z));
            end
            pu = numerator ./ denominator;
        end
        
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
        
        function [v_grid, q_fun] = compute_q_fun(obj, true_theta_idx, gamma, eps)
            % Compute reward function
            % reward_info: gmin,gmax,gnums,g,obs_min,obs_max,obs_val
            r_fun = containers.Map;
            curr_state = obj.reward_info.g;
            g_min = obj.reward_info.gmin;
            g_max = obj.reward_info.gmax;
            g_nums = obj.reward_info.gnums;
            obs_min = obj.reward_info.obs_min;
            obs_max = obj.reward_info.obs_max;
            grid = Grid(g_min, g_max, g_nums);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                next_state = obj.physical_dynamics(curr_state, u_i);
                
                % -inf if moving outside grid
                penalty_outside = (next_state{1} < g_min(1)) | (next_state{2} < g_min(2)) | ...
                    (next_state{1} > g_max(1)) | (next_state{2} > g_max(2));
                penalty_outside(penalty_outside==1) = -inf;
                
                % -inf if moving into obstacle
                penalty_obstacle = (next_state{1} >= obs_min(1)) | (next_state{2} >= obs_min(2)) | ...
                    (next_state{1} <= obs_max(1)) | (next_state{2} <= obs_max(2));
                penalty_obstacle(penalty_obstacle==1) = obj.reward_info.obs_val;
                
                % action costs
                if u_i(1) == 0 && u_i(2) == 0
                    % Stop
                    grid.SetData(ones(size(next_state{1})) .* obj.reward_info.obs_val);
                    grid.SetDataAtReal(obj.reward_info.thetas{true_theta_idx},0);
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
            v_fun_prev = rand(obj.reward_info.gnums);
            j = 0;
            v_grid = Grid(g_min, g_max, g_nums);
            while true
                v_grid.SetData(v_fun_prev);
                possible_vals = zeros([g_nums, numel(obj.controls)]);
                parfor i=1:numel(obj.controls)
                    u_i = obj.controls{i};
                    next_state = obj.physical_dynamics(curr_state, u_i);
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
                j = j + 1;
            end
            v_grid.SetData(v_fun);
            
            % Compute Q-function
            q_fun = containers.Map;
            q_l = zeros([size(v_fun), numel(obj.controls)]);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                next_state = obj.physical_dynamics(curr_state, u_i);
                v_next = v_grid.GetDataAtReal(next_state);
                q_data = r_fun(num2str(u_i)) + (gamma .* v_next);
                q_i = Grid(g_min, g_max, g_nums);
                q_i.SetData(q_data);
                q_fun(num2str(u_i)) = q_i;
                q_l(:,:,i) = q_i.data;
            end
            
            % Make invalid states have Q-value of zero.
            maxQ = max(q_l, [], 3);
            isInvalid = (maxQ == -inf);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                q_i = q_fun(num2str(u_i)).data;
                q_i(isInvalid) = 0;
                q = Grid(g_min, g_max, g_nums);
                q.SetData(q_i);
                q_fun(num2str(u_i)) = q;
            end
            
            % --------- DEBUGGING -------- %
%             controls_text = {'S', 'D', 'U', 'L', 'LD', 'LU', 'R', 'RD', 'RU'};
%             for i=1:obj.num_ctrls
%                 u_i = obj.controls{i};
%                 q_i = q_fun(num2str(u_i)).data;
%                 figure(i)
%                 hold on
%                 s = surf(obj.reward_info.g{1}, obj.reward_info.g{2}, q_i);
% %                 x = obj.reward_info.g{1}(:,1)';
% %                 y = obj.reward_info.g{2}(1,:);
% %                 imagesc(x, y, q_i);
%                 title(strcat('Q(x,u=',controls_text{i},')'));
%                 scatter3(obj.reward_info.thetas{true_theta_idx}{1}, ...
%                         obj.reward_info.thetas{true_theta_idx}{2}, ...
%                         10, ...
%                         'r', 'filled');
%                 pos = [obj.reward_info.obs_min(1), ...
%                         obj.reward_info.obs_min(2), ...
%                         obj.reward_info.obs_max(1)-obj.reward_info.obs_min(1), ...
%                         obj.reward_info.obs_max(2)-obj.reward_info.obs_min(2)]; 
%                 rectangle('Position',pos)
%                 xlim([-4,4]);
%                 ylim([-4,4]);
%                 hold off
%             end
%             bla = 1;
            % --------- DEBUGGING -------- %
        end
    end
end


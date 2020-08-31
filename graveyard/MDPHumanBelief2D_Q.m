classdef MDPHumanBelief2D_Q < handle
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
        
        % Note: Is it safe to assume b scalar or can we have len(theta)>2
        % (which would mean to have b of dimension len(theta)-1)?
    end
    
    methods
        function obj = MDPHumanBelief2D_Q(z0, v_inits, trueThetaIdx, ...
                uThresh, gdisc, gamma, eps)
            % Construct an instance of Grid.
            obj.controls = obj.generate_controls_mdp(gdisc);
            obj.num_ctrls = numel(obj.controls);
            obj.z0 = z0;
            obj.uThresh = uThresh;
            obj.trueThetaIdx = trueThetaIdx;
            obj.v_funs = cell(1,numel(v_inits));
            obj.q_funs = cell(1,numel(v_inits));
            for i=1:numel(v_inits)
                [v_f, q_f] = obj.compute_q_fun(v_inits{i}, gamma, eps);
                obj.v_funs{i} = v_f;
                obj.q_funs{i} = q_f;
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
        
        function [v_grid, q_fun] = compute_q_fun(obj, v_init, gamma, eps)
            % Compute reward function
            r_fun = containers.Map;
            curr_state = v_init.g;
            g_min = v_init.gmin;
            g_max = v_init.gmax;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                next_state = obj.physical_dynamics(curr_state, u_i);
                mask = (next_state{1} < g_min(1)) | (next_state{2} < g_min(2)) | ...
                    (next_state{1} > g_max(1)) | (next_state{2} > g_max(2));
%                 mask(mask==1) = -inf;
                r_i = v_init.GetDataAtReal(next_state); % .* mask;
                r_i(mask==1) = -inf;
                r_fun(num2str(u_i)) = r_i;
            end
            
            % Compute value function
            v_fun_prev = rand(v_init.gnums);
            j = 0;
            v_grid = Grid(v_init.gmin, v_init.gmax, v_init.gnums);
            while true
                v_grid.SetData(v_fun_prev);
                possible_vals = zeros([v_init.gnums, numel(obj.controls)]);
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
                q_i = Grid(v_init.gmin, v_init.gmax, v_init.gnums);
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
                q = Grid(v_init.gmin, v_init.gmax, v_init.gnums);
                q.SetData(q_i);
                q_fun(num2str(u_i)) = q;
            end
        end
    end
end


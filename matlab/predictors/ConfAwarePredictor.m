classdef ConfAwarePredictor
    %CONFAWAREPREDICTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        betas           % (arr) values that beta can take
        grid            % (obj) grid object
        controls        % (cell) real-world controls
        
        goal            % (arr) real-world goal coords in x-y
        goal_idx        % (int) linear index of goal coords
        
        nx              % (int) total number of states
        nu              % (int) total number of controls
        nb              % (int) total number of betas
        
        transition_mat  % (matrix) nx * nx * nu dynamics matrix
        pu_mat          % (matrix) nu * nx * nb action probability matrix
        q_fun 
        q_fun_grid
        
        dyn_sys
    end
    
    methods
        function obj = ConfAwarePredictor(grid, goal, betas, reward_info, gamma, eps)
            %CONFAWAREPREDICTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.betas = betas;
            obj.nb = length(obj.betas);
            obj.grid = grid;
            obj.nu = 9;
            gdisc = (grid.max-grid.min)./(grid.N-1);
            obj.controls = obj.gen_controls(gdisc);
            obj.goal = goal;
            obj.goal_idx = obj.real_to_lin(goal);
            obj.nx = obj.grid.N(1)*obj.grid.N(2);
            obj.grid = grid;
            
            % THIS IS SUCH A BIG HACK TO SAVE ME IMPLEMENTATION TIME.
            b_range = [0.01, 0.99];
            z0 = {1.069, 0, 0.7762}; % doesn't really matter for our purposes.
            obj.dyn_sys = MDPHumanConfidence3D(z0, reward_info, 1, 0.0, gdisc, gamma, eps, betas, b_range);
            obj.q_fun = obj.dyn_sys.q_fun;
            obj.q_fun_grid = reward_info.g;
            
            fprintf('Setting up transition matrix...\n');
            obj.transition_mat = obj.gen_transition_mat();
            fprintf('Setting up action probability matrix...\n');
            obj.pu_mat = obj.gen_pu_mat();
        end
        
        %% Predict for time horizon of T!
        function full_preds = predict(obj, x0, T, pbeta)
            s0 = obj.real_to_lin(x0);
            px0 = zeros(obj.nx, 1);
            px0(s0) = 1;
            
            preds_per_b = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            for bi=1:obj.nb
                preds = cell(1,T+1);
                preds{1} = px0;
                for t=2:T+1
                    pxcurr = zeros(obj.nx, 1);
                    pxprev = preds{t-1};
                    for snext = 1:obj.nx
                        dyn = squeeze(obj.transition_mat(snext, :, :));
                        pub = squeeze(obj.pu_mat(:,:,bi));
                        pub(find(isnan(pub))) = 0; %TMP?
                        sum_dyn_and_pu = sum(dyn .* pub', 2);
                        xprev_dyn_and_pu = sum_dyn_and_pu .* pxprev;
                        pxcurr(snext) = sum(xprev_dyn_and_pu, 1);
                        if pxcurr(snext) ~= 0.
                            bla = 1;
                        end
                    end
                    preds{t} = pxcurr;
                    %pcolor(obj.grid.xs{1}, obj.grid.xs{2}, reshape(pxcurr, obj.grid.N'))
                end
                preds_per_b(num2str(bi)) = preds;
            end
            
            full_preds = cell(1,T+1);
            full_preds{1} = reshape(px0, obj.grid.N'); % Note: assumes dirac dist at initial time.
            
            % marginalize over all betas.
            for t=2:T+1
                pxcurr = zeros(obj.nx, 1);
                for bi=1:obj.nb
                    pxt_beta = preds_per_b(num2str(bi));
                    pxcurr = pxcurr + pbeta(bi) * pxt_beta{t};
                end
                % reshape into 2d grid.
                full_preds{t} = reshape(pxcurr, obj.grid.N');
            end
            
        end
        
        %% Generate P(u_t | x_t, beta) matrix
        %  of size nu * nx * nb
        function pubeta_mat = gen_pu_mat(obj)
            pubeta_mat = zeros(obj.nu, obj.nx, obj.nb);
            
            for ui=1:obj.nu
                for si=1:obj.nx
                    for bi=1:obj.nb
                        pu = obj.pu(si, ui, bi);
                        pubeta_mat(ui, si, bi) = pu;
                    end
                end
            end
            
            total = sum(pubeta_mat(:, 1, 1), 'all');
        end
        
        %% Generate P(x_t+1 | x_t, u_t) matrix 
        %  of size nx * nx * nu
        function transition_mat = gen_transition_mat(obj)
            % Indexed by xnext, xcurr, ucurr 
            transition_mat = zeros(obj.nx, obj.nx, obj.nu);
            for si=1:obj.nx
                for ui=1:obj.nu
                    [snext, ~, ~, isvalid] = obj.dynamics(si, ui);
                    transition_mat(snext, si, ui) = 1;
                    
                    % remove transition if action isn't valid. 
                    if ~isvalid
                        transition_mat(snext, si, ui) = 0;
                    end
                    
                end
            end
        end
        
        %% Dynamics for single state-action!
        function [snext, xnext, ynext, isvalid] = dynamics(obj, si, ui)
            x = obj.grid.xs{1}(si);
            y = obj.grid.xs{2}(si);
            u = obj.controls{ui};
            xnext = x + u(1);
            ynext = y + u(2);
            isvalid = true;
            if xnext > obj.grid.max(1) || xnext < obj.grid.min(1) || ...
                  ynext > obj.grid.max(2) || ynext < obj.grid.min(2)  
            	isvalid = false;  
            end  
            snext = obj.real_to_lin([xnext, ynext]);
        end
        
        %% Computes P(u_t | x_t) \propto e^(-||x_t+1 - goal||_2)
        function pu = pu(obj, si, ui, bi)
            x = obj.grid.xs{1}(si);
            y = obj.grid.xs{2}(si);
            beta = obj.betas(bi);
            [~, xnext, ynext, isvalid] = obj.dynamics(si, ui);
            ureal = obj.controls{ui};
            %qval = -1 * sqrt((xnext - obj.goal(1))^2 + (ynext - obj.goal(2))^2);
            
            % get q-value from our table. 
            qfun = obj.q_fun(num2str(ureal));
            qval = qfun.GetDataAtReal({xnext,ynext});
            %qval = eval_u(obj.q_fun_grid, qfun, [xnext, ynext]);
            
            numer = exp(beta * qval);
            
            % if action is invalid from this state, return 0 prob
            if ~isvalid 
                pu = 0.0;
                return 
            end
            
            % if trying to take the stop action at a non-goal state, return
            % 0 probability.
            if si ~= obj.goal_idx && ui == 1
                pu = 0.0;
                return 
            end
            
            denom = 0.0;
            for uj=1:obj.nu
                [~, xnext, ynext, isvalid] = obj.dynamics(si, uj);
                if ~isvalid
                    continue;
                end
                if si ~= obj.goal_idx && uj == 1
                    continue;
                end
                ureal = obj.controls{uj};
                qfun = obj.q_fun(num2str(ureal));
                qval = qfun.GetDataAtReal({xnext,ynext});
                %qval = -1 * sqrt((xnext - obj.goal(1))^2 + (ynext - obj.goal(2))^2);
                denom = denom + exp(beta * qval);
            end
            
            pu = numer / denom;
        end
        
        %% Generates 9 MDP controls.
        function controls = gen_controls(obj, gdisc)
            controls = cell(1,obj.nu);
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
        
        %% Converts from real to linear idx in grid.
        function lin_idx = real_to_lin(obj, real)
            dist = sqrt((obj.grid.xs{1} - real(1)).^2 + (obj.grid.xs{2} - real(2)).^2);
            [~, lin_idx] = min(dist, [], 'all', 'linear');
        end
        
        %% Updates posterior over betas.
        function posterior = belief_update(obj, xnext, xprev, prior)
            posterior = zeros(size(prior));
            snext = obj.real_to_lin(xnext);
            sprev = obj.real_to_lin(xprev);
            
            dx = xnext(1) - xprev(1);
            dy = xnext(2) - xprev(2);
            
            min_u = NaN;
            min_uidx = -1;
            min_d = 100;
            for i=1:length(obj.controls)
                u = obj.controls{i};
                diff_to_u = sqrt((dx - u(1))^2 + (dy - u(2))^2);
                if diff_to_u < min_d
                    min_d = diff_to_u;
                    min_u = u;
                    min_uidx = i;
                end
            end
            
            % Compute P(u | x, b = 0.1) and P(u | x, b = 1.0)
            pub1 = obj.pu_mat(min_uidx,sprev,1);
            pub2 = obj.pu_mat(min_uidx,sprev,2);
            
            % Update posterior:
            %   P(b = 0.1 | x, u) = 
            %       P(u | x, b = 0.1)*P(b = 0.1)/[P(u | x, b = 0.1)*P(b =
            %                       0.1) + P(u | x, b = 1)*P(b = 1)]
            max_p = 0.99;
            min_p = 0.01;
            posterior(1) = pub1*prior(1)/(pub1*prior(1) + pub2*prior(2));
            posterior(1) = min(max(posterior(1), min_p), max_p);
            posterior(2) = (1 - posterior(1));
        end
        
        %% Plots predictions!
        function plot_preds(obj, preds)
            for t=1:length(preds)
                preds2d = reshape(preds{t}, obj.grid.N');
                %eps = 0.01;
                %bin_preds2d = (preds2d > eps) .* 1.0 + (preds2d <= eps) .* 0.0;
                p = pcolor(obj.grid.xs{1}, obj.grid.xs{2}, preds2d);
                %p.FaceAlpha = 0.5;
                title(strcat('P(x', num2str(t-1), ' | x0)'));
                colorbar
                pause(0.1)
                delete(p)
            end
        end
    end
end


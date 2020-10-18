classdef OptPredictor
    %OPTPREDICTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        grid            % (obj) grid object
        controls        % (cell) real-world controls
        
        goal            % (arr) real-world goal coords in x-y
        nx              % (int) total number of states
        nu              % (int) total number of controls
        q_fun
        q_fun_grid
    end
    
    methods
        function obj = OptPredictor(grid, goal, q_fun, q_fun_grid)
            %OPTPREDICTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.grid = grid;
            obj.nu = 9;
            gdisc = (grid.max-grid.min)./(grid.N-1);
            obj.controls = obj.gen_controls(gdisc);
            obj.goal = goal;
            obj.nx = obj.grid.N(1)*obj.grid.N(2);
            obj.q_fun = q_fun;
            obj.q_fun_grid = q_fun_grid;
        end
        
        %% Predict for time horizon of T!
        function full_preds = predict(obj, x0, T)
            s0 = obj.real_to_lin(x0);
            px0 = zeros(obj.nx, 1);
            px0(s0) = 1;
            
            full_preds = cell(1,T+1);
            full_preds{1} = reshape(px0, obj.grid.N');
            
            x = x0;
            for t=2:T+1
                max_pu = -1;
                opt_ui = NaN;
                for ui=1:obj.nu
                    pu = obj.pu(x, ui);
                    if pu > max_pu
                        max_pu = pu;
                        opt_ui = ui;
                    end
                end
                opt_xnext = obj.dynamics(x, opt_ui);
                opt_snext = obj.real_to_lin(opt_xnext);
                pxcurr = zeros(obj.nx, 1);
                pxcurr(opt_snext) = 1;
                full_preds{t} = reshape(pxcurr, obj.grid.N');
                
                x = opt_xnext;
            end
        end
        
        %% Dynamics for single state-action!
        function xnext = dynamics(obj, x, ui)
            u = obj.controls{ui};
            pxnext = x(1) + u(1);
            pynext = x(2) + u(2);
            xnext = [pxnext, pynext];
        end
        
        %% Computes P(u_t | x_t) \propto e^(-||x_t+1 - goal||_2)
        function pu = pu(obj, x, ui)
            xnext = obj.dynamics(x, ui);
            
            % get q-value from our table. 
            ureal = obj.controls{ui};
            qfun = obj.q_fun(num2str(ureal));
            qval = qfun.GetDataAtReal({xnext(1),xnext(2)});
            
            %qval = -1 * sqrt((xnext(1) - obj.goal(1))^2 + (xnext(2) - obj.goal(2))^2);
            numer = exp(qval);
            
            denom = 0.0;
            for uj=1:obj.nu
                xnext = obj.dynamics(x, uj);
                ureal = obj.controls{ui};
                qfun = obj.q_fun(num2str(ureal));
                qval = qfun.GetDataAtReal({xnext(1),xnext(2)});
                %qval = -1 * sqrt((xnext(1) - obj.goal(1))^2 + (xnext(2) - obj.goal(2))^2);
                denom = denom + exp(qval);
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
        
        %% plot the preds.
        function plot_preds(obj, preds)
            for t=1:length(preds)                
                p2d = reshape(preds{t}, obj.grid.N');
                pcolor(obj.grid.xs{1}, obj.grid.xs{2}, p2d);
                title(strcat('P(x', num2str(t-1), ' | x0)'));
                colorbar
                pause(0.1)
            end
        end
        
        %% Converts from real to linear idx in grid.
        function lin_idx = real_to_lin(obj, real)
            dist = sqrt((obj.grid.xs{1} - real(1)).^2 + (obj.grid.xs{2} - real(2)).^2);
            [~, lin_idx] = min(dist, [], 'all', 'linear');
        end
    end
end


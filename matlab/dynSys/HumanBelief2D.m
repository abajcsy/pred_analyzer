classdef HumanBelief2D < handle
    % Human belief 2D class
    
    properties
        dt % discrete timestep
        thetas % set of model parameters
        num_ctrls % number of discrete controls
        controls % discretized controls
        z0 % initial joint state (x, b)
        b_range         % (arr) range of belief values to clip to.
        
        % Note: Is it safe to assume b scalar or can we have len(theta)>2
        % (which would mean to have b of dimension len(theta)-1)?
    end
    
    methods
        function obj = HumanBelief2D(dt, thetas, num_ctrls, controls, z0, b_range)
            % Construct an instance of Grid.
            obj.dt = dt;
            obj.thetas = thetas;
            obj.num_ctrls = num_ctrls;
            obj.controls = controls;
            obj.z0 = z0;
            obj.b_range = b_range;
            
        end
        
        function znext = dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(size(z));
            znext{1} = z{1} + obj.dt.*u;
            znext{2} = obj.belief_update(u,z{1},z{2});
        end
        
        function bnext = belief_update(obj,u,x,b)
            % Calculate Bayesian posterior update.
            b0 = obj.pugivenxtheta(u, x, obj.thetas(1)) .* b;
            b1 = obj.pugivenxtheta(u, x, obj.thetas(2)) .* (1-b);
            normalizer = b0 + b1;
            bnext = b0 ./ normalizer;
            bnext = min(max(bnext, obj.b_range(1)), obj.b_range(2));
        end
        
        function pu = pugivenxtheta(obj,u,x,theta)
            % Return probability of control u given state x and model parameter theta.
            x_next = x + obj.dt.*u;
            numerator = exp(-abs(x_next - theta));
            denominator = 0;
            for i=1:obj.num_ctrls
                u_i = obj.controls(i);
                x_next_i = x + obj.dt*u_i;
                denominator = denominator + exp(-abs(x_next_i - theta));
            end
            pu = numerator ./ denominator;
        end
    end
end


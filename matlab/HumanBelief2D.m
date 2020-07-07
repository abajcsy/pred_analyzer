classdef HumanBelief2D < handle
    % Human belief 1D class
    
    properties
        dt % discrete timestep
        thetas % set of model parameters
        num_ctrls % number of discrete controls
        controls % discretized controls
        z0 % initial joint state (x, b)
        v % constant speed of human
        uThresh % threshold probability for sufficiently likely controls
        
        % Note: Is it safe to assume b scalar or can we have len(theta)>2
        % (which would mean to have b of dimension len(theta)-1)?
    end
    
    methods
        function obj = HumanBelief2D(dt, thetas, num_ctrls, controls, z0, v, uThresh)
            % Construct an instance of Grid.
            obj.dt = dt;
            obj.thetas = thetas;
            obj.num_ctrls = num_ctrls;
            obj.controls = controls;
            obj.z0 = z0;
            obj.v = v;
            obj.uThresh = uThresh;
        end
        
        function znext = dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(size(z));
            znext{1} = z{1} + obj.dt .* (obj.v .* cos(u));
            znext{2} = z{2} + obj.dt .* (obj.v .* sin(u));
            znext{3} = obj.belief_update(u,z);
        end
        
        function bnext = belief_update(obj,u,z)
            % Calculate Bayesian posterior update.
            b0 = obj.pugivenxtheta(u, z, obj.thetas{1}) .* z{3};
            b1 = obj.pugivenxtheta(u, z, obj.thetas{2}) .* (1-z{3});
            normalizer = b0 + b1;
            bnext = b0 ./ normalizer;
        end
        
        function likelyMasks = getLikelyMasks(obj, z)
            likelyMasks = containers.Map;
            for i=1:obj.num_ctrls
                u_i = obj.controls(i);
                b0 = obj.pugivenxtheta(u_i, z, obj.thetas{1}) .* z{3};
                b1 = obj.pugivenxtheta(u_i, z, obj.thetas{2}) .* (1-z{3});
                normalizer = b0 + b1;
                
                mask = (normalizer >= obj.uThresh);
                mask = mask * 1;
                mask(mask == 0) = nan;
                likelyMasks(num2str(u_i)) = mask;
            end
        end
        
        function pu = pugivenxtheta(obj,u,z,theta)
            % Return probability of control u given state x and model parameter theta.
            x_next = z{1} + obj.dt .* (obj.v .* cos(u));
            y_next = z{2} + obj.dt .* (obj.v .* sin(u));
            numerator = exp(-((x_next - theta(1)).^2 + (y_next - theta(2)).^2).^0.5);
            denominator = 0;
            for i=1:obj.num_ctrls
                u_i = obj.controls(i);
                x_next_i = z{1} + obj.dt .* (obj.v .* cos(u_i));
                y_next_i = z{2} + obj.dt .* (obj.v .* sin(u_i));
                denominator = denominator + exp(-((x_next_i - theta(1)).^2 + (y_next_i - theta(2)).^2).^0.5);
            end
            pu = numerator ./ denominator;
        end
    end
end


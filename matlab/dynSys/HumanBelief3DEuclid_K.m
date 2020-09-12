classdef HumanBelief3DEuclid_K < handle
    % Human belief 1D class
    
    properties
        dt % discrete timestep
        thetas % set of model parameters
        num_ctrls % number of discrete controls
        controls % discretized controls
        z0 % initial joint state z = (x, y, b(theta_1))
        v % constant speed of human
        uThresh % threshold for sufficiently likely controls (top k)
        trueThetaIdx % true human intent parameter. 
        b_range         % (arr) range of belief values to clip to.
        
        % Note: Is it safe to assume b scalar or can we have len(theta)>2
        % (which would mean to have b of dimension len(theta)-1)?
    end
    
    methods
        function obj = HumanBelief3DEuclid_K(z0, thetas, trueThetaIdx, uThresh, ...
                dt, v, num_ctrls, b_range)
            % Construct an instance of Grid.
            obj.dt = dt;
            obj.thetas = thetas;
            obj.num_ctrls = num_ctrls;
            obj.v = v;
            obj.controls = obj.generate_controls_non_mdp(obj.num_ctrls, obj.v);
            obj.z0 = z0;
            obj.uThresh = uThresh;
            obj.trueThetaIdx = trueThetaIdx;
            obj.b_range = b_range;
        end
        
        function znext = dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(size(z));
            znext{1} = z{1} + obj.dt .* u(1);
            znext{2} = z{2} + obj.dt .* u(2);
            znext{3} = obj.belief_update(u,z);
        end
        
        function bnext = belief_update(obj,u,z)
            % Calculate Bayesian posterior update.
            b0 = obj.pugivenxtheta(u, z, obj.thetas{1}) .* z{3};
            b1 = obj.pugivenxtheta(u, z, obj.thetas{2}) .* (1-z{3});
            normalizer = b0 + b1;
            bnext = b0 ./ normalizer;
            bnext = min(max(bnext, obj.b_range(1)), obj.b_range(2));
        end
        
        function likelyMasks = getLikelyMasks(obj, z)
            if numel(size(z{1})) == 3
                grid = true;
            else
                grid = false;
            end
            
            likelyMasks = containers.Map;
            
            probs = zeros([size(z{1}), numel(obj.controls)]);
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                putheta = obj.pugivenxtheta(u_i, z, obj.thetas{obj.trueThetaIdx});
                if grid
                    probs(:,:,:,i) = putheta;
                else
                    probs(:,i) = putheta;
                end
            end
            
            if grid
                [~, IX] = sort(probs,4);
                [~,IX]=sort(IX,4);
            else
                [~, IX] = sort(probs,2);
                [~,IX]=sort(IX,2);
            end
            
            mask = (IX > numel(obj.controls) - obj.uThresh);
            mask = mask * 1.0;
            mask(mask==0) = nan;
            
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                if grid
                    likelyMasks(num2str(u_i)) = mask(:,:,:,i);
                else
                    likelyMasks(num2str(u_i)) = mask(:,i);
                end
            end
        end
        
        function pu = pugivenxtheta(obj,u,z,theta)
            % Return probability of control u given state x and model parameter theta.
            x_next = z{1} + obj.dt .* u(1);
            y_next = z{2} + obj.dt .* u(2);
            numerator = exp(-((x_next - theta(1)).^2 + (y_next - theta(2)).^2).^0.5);
            denominator = 0;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                x_next_i = z{1} + obj.dt .* u_i(1);
                y_next_i = z{2} + obj.dt .* u_i(2);
                denominator = denominator + exp(-((x_next_i - theta(1)).^2 + (y_next_i - theta(2)).^2).^0.5);
            end
            pu = numerator ./ denominator;
        end
        
        function controls = generate_controls_non_mdp(obj, n, v)
            us = linspace(0,2*pi - 1e-2,n);
            controls = cell(1,n);

            for i=1:n
                u_i = us(i);
                controls{i} = [v*cos(u_i), v*sin(u_i)];
            end
        end
    end
end


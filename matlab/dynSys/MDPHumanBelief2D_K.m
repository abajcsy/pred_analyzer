classdef MDPHumanBelief2D_K < handle
    % Human belief 1D class
    
    properties
        thetas % set of model parameters
        num_ctrls % number of discrete controls
        controls % discretized controls
        z0 % initial joint state z = (x, y, b(theta_1))
        uThresh % threshold probability for sufficiently likely controls
        trueThetaIdx % true human intent parameter. 
        
        % Note: Is it safe to assume b scalar or can we have len(theta)>2
        % (which would mean to have b of dimension len(theta)-1)?
    end
    
    methods
        function obj = MDPHumanBelief2D_K(z0, thetas, trueThetaIdx, ...
                uThresh, gdisc)
            % Construct an instance of Grid.
            obj.thetas = thetas;
            obj.controls = obj.generate_controls_mdp(gdisc);
            obj.num_ctrls = numel(obj.controls);
            obj.z0 = z0;
            obj.uThresh = uThresh;
            obj.trueThetaIdx = trueThetaIdx;
        end
        
        function znext = dynamics(obj,z,u)
            % Return next state after applying control u to z. 
            % Note that no interpolation is done (should be handled by Grid class).
            znext = cell(size(z));
            znext{1} = z{1} + u(1);
            znext{2} = z{2} + u(2);
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
                mask = (sort(probs,4) > numel(obj.controls) - obj.uThresh);
            else
                mask = (sort(probs,2) > numel(obj.controls) - obj.uThresh);
            end
            
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
            x_next = z{1} + u(1);
            y_next = z{2} + u(2);
            numerator = exp(-((x_next - theta(1)).^2 + (y_next - theta(2)).^2).^0.5);
            denominator = 0;
            for i=1:obj.num_ctrls
                u_i = obj.controls{i};
                x_next_i = z{1} + u_i(1);
                y_next_i = z{2} + u_i(2);
                denominator = denominator + exp(-((x_next_i - theta(1)).^2 + (y_next_i - theta(2)).^2).^0.5);
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
    end
end


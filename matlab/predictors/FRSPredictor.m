classdef FRSPredictor
    %FRSPREDICTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        grid            % (obj) grid object
        controls        % (cell) real-world controls
        
        nx              % (int) total number of states
        nu              % (int) total number of controls
    end
    
    methods
        function obj = FRSPredictor(grid)
            %FRSPREDICTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.grid = grid;
            obj.nx = obj.grid.N(1)*obj.grid.N(2);
        end
        
        %% Predict for time horizon of T!
        function full_preds = predict(obj, x0, T)
            s0 = obj.real_to_lin(x0);
            px0 = zeros(obj.nx, 1);
            px0(s0) = 1;
            full_preds = cell(1,T+1);
            
            % Reshape into 2D matrix of predictions.
            full_preds{1} = reshape(px0, obj.grid.N');
            
            for t=2:T+1
                next_preds = imdilate(full_preds{t-1},ones(3));
                full_preds{t} = next_preds;
            end
        end
        
        %% Converts from real to linear idx in grid.
        function lin_idx = real_to_lin(obj, real)
            dist = sqrt((obj.grid.xs{1} - real(1)).^2 + (obj.grid.xs{2} - real(2)).^2);
            [~, lin_idx] = min(dist, [], 'all', 'linear');
        end
        
        %% Plot
        function plot_preds(obj, preds)
            for t=1:length(preds)
                preds2d = reshape(preds{t}, obj.grid.N');
                %eps = 0.01;
                %bin_preds2d = (preds2d > eps) .* 1.0 + (preds2d <= eps) .* 0.0;
                pcolor(obj.grid.xs{1}, obj.grid.xs{2}, preds2d);
                title(strcat('P(x', num2str(t-1), ' | x0)'));
                colorbar
                pause(0.1)
            end
        end
    end
end


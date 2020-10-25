classdef Grid < handle
    % Grid class
    
    properties
        gmin % minimum real-world coordinate of ND grid in all directions
        gmax % maximum real-world coordinate of ND grid in all directions
        gnums % number of grid points per dimension
        gdisc % resolution in each dimension (computed by (grid_max(i) - grid_min(i)) ./ (gnums(i)-1))
        data % flattened 1D version of ND grid (in row-major order)
        g % meshgrid containing grid points
    end
    
    methods
        function obj = Grid(gmin, gmax, gnums)
            % Construct an instance of Grid.
            obj.gmin = gmin;
            obj.gmax = gmax;
            obj.gnums = gnums;
            obj.gdisc = (gmax - gmin) ./ (gnums-1);
            if size(gnums) == 1
                obj.data = zeros(1,gnums);
            else
                if ~isrow(gnums)
                    obj.gnums = gnums';
                    obj.gmax = gmax';
                    obj.gmin = gmin';
                end
                obj.data = zeros(obj.gnums);
            end
            
            [~, dims] = size(obj.gnums);
            grid_els = cell(1, dims);
            for dim=1:dims
                grid_els{dim} = obj.gmin(dim):obj.gdisc(dim):obj.gmax(dim);
            end
            g = cell(1,numel(grid_els));
            [g{:}] = ndgrid(grid_els{:});
            obj.g = g;
        end
        
        function coord = RealToCoords(obj,real)
            % Return coordinates from real value.
            [~, dims] = size(obj.gnums);
            coord = cell(1, dims);
            parfor dim=1:dims
                gridding = obj.gmin(dim):obj.gdisc(dim):obj.gmax(dim);
                coord{dim} = interp1(gridding,gridding,real{dim},'nearest','extrap');
            end
        end
        
        function grid = get_grid(obj)
            grid = obj.g;
        end
        
        function idx = CoordsToIdx(obj,coords)
            % Return linear index from grid coordinates
            [~, dims] = size(obj.gnums);
            index = cell(1, dims);
            parfor dim=1:dims
                index{dim} = int32(((coords{dim} - obj.gmin(dim)) ./ obj.gdisc(dim)) + 1);
            end
            
            idx = cell(size(coords{1}));
%             parfor ind=1:numel(idx)
            parfor ind=1:numel(idx)
                sub = [];
                for dim=1:dims
                    sub = [sub index{dim}(ind)];
                end
                sub = num2cell(sub);
                new_ind = sub2ind(size(obj.data), sub{:});
                idx{ind} = new_ind;
%                 try
%                     new_ind = sub2ind(size(obj.data), sub{:});
%                     idx{ind} = new_ind;
%                 catch
%                     idx{ind} = nan;
%                 end
            end
        end
        
        function data = GetDataAtReal(obj,real)
            % Return data at real coordinates.
            idx = obj.CoordsToIdx(obj.RealToCoords(real));
            data = zeros(size(real{1}));
            parfor data_ind=1:numel(real{1})
                corr_ind = idx{data_ind};
                if isnan(corr_ind) && false
                    data(data_ind) = nan;
                else
                    data(data_ind) = obj.data(corr_ind);
                end
            end
        end
        
        function idx = RealToIdx(obj,real)
            % Return data at real coordinates.
            idx = obj.CoordsToIdx(obj.RealToCoords(real));
        end
        
        function SetData(obj,newdata)
            % Change data property.
            obj.data = newdata;
        end
        
        function SetDataAtReal(obj,real,val)
            % Set data at coordinates defined by real
            idx = obj.CoordsToIdx(obj.RealToCoords(real));
            for data_ind=1:numel(real{1})
                obj.data(idx{data_ind}) = val(data_ind);
            end
        end
        
        % Generalize to n-dims, only implemented for n=3 currently.
        function val = interpolate(obj,state)
            coords = obj.RealToCoords(state);
            grid_coords = cell(1,2^3);
            alphas = cell(1,2^3);

            % Find upper/lower bounds of box surrounding (ungridded) next
            % state and alpha values for each dimension
            for l=1:3
                if coords{l} < state{l}
                    other = coords{l} + obj.gdisc(l);
                    grid_coords{l} = [coords{l}, other];
                elseif coords{l} > state{l}
                    other = coords{l} - obj.gdisc(l);
                    grid_coords{l} = [other, coords{l}];
                else
                    other = coords{l} - grid.gdisc(l);
                    if other <= obj.gmin(l)
                        grid_coords{l} = [coords{l}, coords{l} + obj.gdisc(l)];
                    else
                        grid_coords{l} = [other, coords{l}];
                    end
                end
                alphas{l} = 1 - (state{l} - grid_coords{l}(1))/obj.gdisc(l);
            end

            val = 0;
            for a=1:2
                if a == 1
                    w_a = alphas{1};
                else
                    w_a = 1 - alphas{1};
                end

                for b=1:2
                    if b == 1
                        w_b = alphas{2};
                    else
                        w_b = 1 - alphas{2};
                    end

                    for c=1:2
                        if c == 1
                            w_c = alphas{3};
                        else
                            w_c = 1 - alphas{3};
                        end

                        weight = w_a*w_b*w_c;

                        vertex = {grid_coords{1}(a),grid_coords{2}(b),grid_coords{3}(c)};
                        vertex_idx = obj.RealToIdx(vertex);
                        val = val + weight*obj.data(vertex_idx{1});
                    end
                end
            end
        end
    end
end


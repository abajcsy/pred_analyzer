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
                obj.data = zeros(gnums);
            end
            
            [~, dims] = size(obj.gnums);
            grid_els = cell(1, dims);
            parfor dim=1:dims
                grid_els{dim} = obj.gmin(dim):obj.gdisc(dim):obj.gmax(dim);
            end
            g = cell(1,numel(grid_els));
            [g{:}] = ndgrid(grid_els{:});
            obj.g = g;
        end
        
        % TODO: Is the Euclidean distance metric better or the same as
        % Manhattan on a grid?
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
            % TODO: Turn idx into containing linear index. Loop through all
            % elements? TRY parfor?
            idx = cell(size(coords{1}));
            parfor ind=1:numel(idx)
                sub = [];
                for dim=1:dims
                    sub = [sub index{dim}(ind)];
                end
                sub = num2cell(sub);
                new_ind = sub2ind(size(obj.data), sub{:});
                idx{ind} = new_ind;
            end
        end
        
        function data = GetDataAtReal(obj,real)
            % Return data at real coordinates.
            idx = obj.CoordsToIdx(obj.RealToCoords(real));
            data = zeros(size(real{1}));
            parfor data_ind=1:numel(real{1})
                data(data_ind) = obj.data(idx{data_ind});
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
            % TODO: Finish this.
            idx = obj.CoordsToIdx(obj.RealToCoords(real));
            for data_ind=1:numel(real{1})
                obj.data(idx{data_ind}) = val(data_ind);
            end
        end
    end
end


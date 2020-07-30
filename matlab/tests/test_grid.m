%% This is a test file for the Grid class

% TODO: Update 1D and 2D grid tests with different data setting (use matrix
% instead of cell array.

% %% 1D Grid Tests
% gmin = [-2];
% gmax = [3];
% gnums = [6];
% grid = Grid(gmin, gmax, gnums);
% 
% a = cell(1,1);
% a{1} = [-3];
% grid.RealToCoords(a) % -2
% 
% a = cell(1,1);
% a{1} = [2.1];
% grid.RealToCoords(a) % 2
% 
% a = cell(1,1);
% a{1} = [-2];
% grid.CoordsToIdx(a) % 1
% 
% a = cell(1,1);
% a{1} = [2];
% grid.CoordsToIdx(a) % 5
% 
% a = cell(1,1);
% a{1} = [2, 3];
% d = cell(1,2);
% d{1} = 10;
% d{2} = 11;
% grid.SetDataAtReal(a, d);
% celldisp(grid.GetDataAtReal(a)) % 10, 11

% %% 2D Grid Tests
% gmin = [-2,-2];
% gmax = [2,3];
% gnums = [5,6];
% grid = Grid(gmin, gmax, gnums);
% 
% a = cell(1,2);
% a{1} = [-3, 1.9];
% a{2} = [-3, 2.9];
% celldisp(grid.RealToCoords(a)) % -2 -2 , 2 3
% 
% a = cell(1,2);
% a{1} = [-2, 2];
% a{2} = [-2, 3];
% celldisp(grid.CoordsToIdx(a)) % 1, 30
% 
% a = cell(1,2);
% a{1} = [-3, 2];
% a{2} = [-2, 3];
% d = cell(1,2);
% d{1} = 10;
% d{2} = 12;
% grid.SetDataAtReal(a, d);
% celldisp(grid.GetDataAtReal(a)) % 10, 12

%% 3D Grid Tests
gmin = [-2,-2,-4];
gmax = [2,3,4];
gnums = [5,6,9];
grid = Grid(gmin, gmax, gnums);

a = cell(1,3);
a{1} = [-3, 1.9];
a{2} = [-3, 2.9];
a{3} = [-2.1, 1.1];
celldisp(grid.RealToCoords(a)) % -2 -2 -2, 2 3 1

a = cell(1,3);
a{1} = [-2, 2];
a{2} = [-2, 3];
a{3} = [-4, 4];
celldisp(grid.CoordsToIdx(a)) % 1, 270

a = cell(1,3);
a{1} = [-2, 2, 1];
a{2} = [-2, 3, 2];
a{3} = [-4, 4, 3];
d = zeros(1,3);
d(1) = 10;
d(2) = 12;
d(3) = 20;
grid.SetDataAtReal(a, d);
grid.GetDataAtReal(a) % 10, 12, 20
%% This is a simple test file for running the fast marching method (FMM)
% for computing the signed distance to an arbitrarily-shaped set.

% Create simple 2D grid. 
grid_low = [-2,0];
grid_up = [2,1];
N = [9, 9];
grid = createGrid(grid_low, grid_up, N);

% A 2D target set with (+1) for outside of target
% region and (-1) for inside target.
target_set = ones(size(grid.xs{1}));

% Option 1: this target set does include boundary of grid.
%target_set(:,7:9) = -1; 

% Option 2: this target set representation doesn't include boundary of grid.
target_set(2:8,8) = -1; 

% Compute signed distance, i.e. l(x), for this target set.
signed_dist = compute_fmm_map(grid, target_set);

% Plot signed distance function, l(x). 
surf(grid.xs{1}, grid.xs{2}, signed_dist);
zlabel('$l(z)$', 'Interpreter', 'Latex');
xlabel('$x$', 'Interpreter', 'Latex');
ylabel('$b(\theta)$', 'Interpreter', 'Latex');
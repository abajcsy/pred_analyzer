clc
clf
clear all

%% This is a simple test file for running the fast marching method (FMM)
% for computing the signed distance to an arbitrarily-shaped set.

% Create simple 2D grid. 
grid_low = [-2,0];
grid_up = [2,1];
N = [10, 10];
grid = createGrid(grid_low, grid_up, N);

%% Goal locations (unknown).
g1 = -2;
g2 = 2;
goals = [g1, g2];
true_goal = g1;

%% Other
dt = 0.1;
timesteps = 5;

%% Set of controls.
ctrls = [-1,1]; % -1 == LEFT, 1 == RIGHT.

% A 2D target set with (+1) for outside of target
% region and (-1) for inside target.
target_set = ones(size(grid.xs{1}));

% Option 1: this target set does include boundary of grid.
%target_set(:,7:9) = -1; 

% Option 2: this target set representation doesn't include boundary of grid.
target_set(2:19,17:19) = -1; 

% Compute signed distance, i.e. l(x), for this target set.
value_funcs(:,:,1) = compute_fmm_map(grid, target_set);

% belief_update(1, -1, 0.2, goals, dt)
% pugiveng(1, -1, goals(1), goals, dt)
% pugiveng(1, -1, goals(2), goals, dt)

% Grid values for each axis.
x_roundTargets = linspace(grid_low(1),grid_up(1),N(1));
b_roundTargets = linspace(grid_low(2),grid_up(2),N(2));

% Check that x_to_index and select_by_ind work on initial value_func.
% Expect results to be the same.
% ind = cell(2);
% ind{1} = x_to_index(grid.xs{1}, (grid_up(1)-grid_low(1))/(N(1)-1), grid_low(1));
% ind{2} = x_to_index(grid.xs{2}, (grid_up(2)-grid_low(2))/(N(2)-1), grid_low(2));
% 
% ind{1}
% grid.xs{1}
% 
% value_funcs(:,:,1)-select_by_ind(value_funcs(:,:,1), ind)

% Find possible indices that controls causes states to move to.
ind_possible = cell(2);
ind_possible{1} = ind_next(ctrls(1),grid.xs,goals,ctrls,dt,x_roundTargets,b_roundTargets, grid_low, grid_up, N);
ind_possible{2} = ind_next(ctrls(2),grid.xs,goals,ctrls,dt,x_roundTargets,b_roundTargets, grid_low, grid_up, N);

for i=1:timesteps
    value_func_next = value_funcs(:,:,i);
    value_func_curr_1 = select_by_ind(value_func_next,ind_possible{1});
    value_func_curr_2 = select_by_ind(value_func_next,ind_possible{2});
    v_possible(:,:,1) = value_func_curr_1;
    v_possible(:,:,2) = value_func_curr_2;
    value_func_curr = min(v_possible, [], 3);
    value_funcs(:,:,i+1) = value_func_curr;
end

for i=1:timesteps+1
    % Plot signed distance function, l(x). 
    h = subplot(1,timesteps+1,i);
%     figure;
    v = (value_funcs(:,:,i) > 0);
    surf(grid.xs{1}, grid.xs{2}, double(v));
    zlabel('$l(z)$', 'Interpreter', 'Latex');
    xlabel('$x$', 'Interpreter', 'Latex');
    ylabel('$b(\theta)$', 'Interpreter', 'Latex');
end

function A_select = select_by_ind(A, ind)
    [m,n] = size(A);
    A_select = zeros(size(A));
    for i=1:m
        for j=1:n
            i_select = int32(ind{1}(i,j));
            j_select = int32(ind{2}(i,j));
            A_select(i,j) = A(i_select,j_select);
        end
    end
end

function index = x_to_index(x, step, min)
    x = x - min;
    index = (x ./ step) + 1;
end

function ind = ind_next(u, xs, goals, ctrls, dt, x_roundTargets, b_roundTargets, grid_low, grid_up, N)
    x = xs{1} + dt*u;
    b = xs{2};
    % Double check belief_update function is correctly implemented.
    b = belief_update(u, xs{1}, b, goals, ctrls, dt);
    
    % Double check what interp1 does.
    % Map states to grid values
    x = interp1(x_roundTargets,x_roundTargets,x,'nearest','extrap');
    b = interp1(b_roundTargets,b_roundTargets,b,'nearest','extrap');
    
    % Map grid values to indices
    ind = cell(2);
    ind{1} = x_to_index(x, (grid_up(1)-grid_low(1))/(N(1)-1), grid_low(1));
    ind{2} = x_to_index(b, (grid_up(2)-grid_low(2))/(N(2)-1), grid_low(2));
end

function bnext = belief_update(u, x, b, goals, ctrls, dt)
    b0 = pugiveng(u, x, goals(1), ctrls, dt) .* b;
    b1 = pugiveng(u, x, goals(2), ctrls, dt) .* (1-b);
    normalizer = b0 + b1;
    bnext = b0 ./ normalizer;
end

function pu = pugiveng(u, x, g, ctrls, dt)
    x_next_1 = x + ctrls(1)*dt;
    x_next_2 = x + ctrls(2)*dt;
    numerator = exp(-abs(x + dt*u - g));
    denominator = exp(-abs(x_next_1 - g)) + exp(-abs(x_next_2 - g));
    pu = numerator ./ denominator;
end
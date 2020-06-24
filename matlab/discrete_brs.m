clear

%% This is a simple test file for running the fast marching method (FMM)
% for computing the signed distance to an arbitrarily-shaped set.

% Create simple 2D grid. 
grid_low = [-4,0];
grid_up = [4,1];
N = [20, 20];
grid = createGrid(grid_low, grid_up, N);

%% Goal locations (unknown).
g1 = -2;
g2 = 2;
goals = [g1, g2];
true_goal = g1;

%% Other
dt = 0.2;
timesteps = 5;

%% Set of controls.
ctrls = [-1,1]; % -1 == LEFT, 1 == RIGHT.

% Grid values for each axis.
x_roundTargets = linspace(grid_low(1),grid_up(1),N(1));
b_roundTargets = linspace(grid_low(2),grid_up(2),N(2));

% A 2D target set with (+1) for outside of target
% region and (-1) for inside target.
target_set = ones(size(grid.xs{1}));

% Option 1: this target set does include boundary of grid.
%target_set(:,7:9) = -1; 

% Option 2: this target set representation doesn't include boundary of grid.
x_target_ind = [2,N(1)-1];
b_target_ind = [17,N(2)-1];
target_set(x_target_ind(1):x_target_ind(2),b_target_ind(1):b_target_ind(2)) = -1;

% Values corresponding to target set
x_target = [x_roundTargets(x_target_ind(1)),x_roundTargets(x_target_ind(2))];
b_target = [b_roundTargets(b_target_ind(1)),b_roundTargets(b_target_ind(2))];

% Compute signed distance, i.e. l(x), for this target set.
value_funcs(:,:,1) = compute_fmm_map(grid, target_set);

% belief_update(1, -1, 0.2, goals, dt)
% pugiveng(1, -1, goals(1), goals, dt)
% pugiveng(1, -1, goals(2), goals, dt)

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

% % Plot valued functions
% for i=1:timesteps+1
%     % Plot signed distance function, l(x). 
%     h = subplot(1,timesteps+1,i);
% %     figure;
%     v = (value_funcs(:,:,i) > 0);
%     surf(grid.xs{1}, grid.xs{2}, double(v));
%     zlabel('$l(z)$', 'Interpreter', 'Latex');
%     xlabel('$x$', 'Interpreter', 'Latex');
%     ylabel('$b(\theta)$', 'Interpreter', 'Latex');
% end

% Check states in BRS (negative valued states of final value function) can reach belief set in T timesteps
[row, col] = find(value_funcs(:,:,timesteps+1)<0);
valid = [];
for i=1:length(row)
    ind = [row(i), col(i)];
    state = [x_roundTargets(ind(1)), b_roundTargets(ind(2))];
    ended = 1;
    for t=1:timesteps+1
        if (state(1) <= x_target(2)) && (state(1) >= x_target(1)) && (state(2) <= b_target(2)) && (state(2) >= b_target(1))
            ended = 0;
        end
        x_next = interp1(x_roundTargets,x_roundTargets,state(1) - 1*dt,'nearest','extrap');
        b_next = interp1(b_roundTargets,b_roundTargets,belief_update(-1, state(1), state(2), goals, ctrls, dt),'nearest','extrap');
        state = [x_next, b_next];
    end
    valid = [ended valid];
end

sum(valid)

% Find optimal control sequence
state = [0, 0.5];
ctrl_seq = [];

for t=1:timesteps
    possible_state = cell(2);
    possible_ind = cell(2);
    for i=1:length(ctrls)
        u = ctrls(i);
        x_new = state(1) + u*dt;
        b_new = belief_update(u, state(1), state(2), goals, ctrls, dt);
        state_new = [x_new, b_new];
        state_new = state_to_grid(state_new, x_roundTargets, b_roundTargets);
        possible_state{i} = state_new;
        possible_ind{i} = state_to_index(state_new,grid_low,grid_up,N);
    end
    ctrl_index = 2;
    if value_funcs(possible_ind{1}(1),possible_ind{1}(2), timesteps + 1 - t) < value_funcs(possible_ind{2}(1),possible_ind{2}(2), timesteps + 1 - t)
        ctrl_index = 1;
    end
    ctrl_seq = [ctrls(ctrl_index) ctrl_seq];
    state = possible_state{ctrl_index};
    if (state(1) <= x_target(2)) && (state(1) >= x_target(1)) && (state(2) <= b_target(2)) && (state(2) >= b_target(1))
        disp('Reached target set')
        state
        ctrl_seq
        break
    end
end

function state_grid = state_to_grid(state, x_roundTargets, b_roundTargets)
    state_grid = [interp1(x_roundTargets,x_roundTargets,state(1),'nearest','extrap'), interp1(b_roundTargets,b_roundTargets,state(2),'nearest','extrap')];
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

function index = state_to_index(state, grid_low, grid_up, N)
    ind1 = x_to_index(state(1), (grid_up(1)-grid_low(1))/(N(1)-1), grid_low(1));
    ind2 = x_to_index(state(2), (grid_up(2)-grid_low(2))/(N(2)-1), grid_low(2));
    index = [int32(ind1), int32(ind2)];
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
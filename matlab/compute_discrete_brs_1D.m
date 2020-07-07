clear
%% File to compute reachable sets and optimal control via discrete HJ

% Grid setup
gmin = [-4,0];
gmax = [4,1];
gnums = [20,20];

% Hamilton-Jacobi Problem Setup
uMode = "min"; % min or max
num_timesteps = 20;
plot = false;

% Joint Dynamics Setup
dt = 0.8;
thetas = [-2, 2];
num_ctrls = 2;
controls = [-1, 1];

z0 = [0, 0.5];
dyn_sys = HumanBelief1D(dt, thetas, num_ctrls, controls, z0);

% Target Set Setup
x_target_ind = [2,gnums(1)-1];
b_target_ind = [17,gnums(2)];
initial_value_fun = construct_value_fun(x_target_ind, b_target_ind, gmin, gmax, gnums);
tic
% Store initial value function
value_funs = cell(1, num_timesteps);
value_funs{end} = initial_value_fun;

% Compute BRS backwards in time
tidx = num_timesteps - 1;

while tidx > 0
    % Create grid and set data to value function at t+1
    grid = Grid(gmin, gmax, gnums);
    grid.SetData(value_funs{tidx + 1});
    
    % Compute possible next state after one timestep and find possible
    % value functions
    current_state = grid.get_grid();
    possible_value_funs = zeros([gnums, numel(controls)]);
    for i=1:numel(controls)
        u_i = controls(i);
        next_state = dyn_sys.dynamics(current_state, u_i);
        possible_value_funs(:,:,i) = grid.GetDataAtReal(next_state);
    end
    
    % Minimize/Maximize over possible future value functions.
    if strcmp(uMode, "min")
        value_fun = min(possible_value_funs, [], 3);
    else
        value_fun = max(possible_value_funs, [], 3);
    end
    
    value_funs{tidx} = value_fun;
    
    tidx = tidx - 1;
end

toc

% Plot valued functions
if plot
    g = grid.get_grid();
    for i=1:num_timesteps
        % Plot signed distance function, l(x). 
        h = subplot(1,num_timesteps,i);
    %     figure;
        v = (value_funs{i} > 0);
        surf(g{1}, g{2}, double(v));
        zlabel('$l(z)$', 'Interpreter', 'Latex');
        xlabel('$x$', 'Interpreter', 'Latex');
        ylabel('$b(\theta)$', 'Interpreter', 'Latex');
    end
end

initial_state = cell(1,2);
initial_state{1} = 0;
initial_state{2} = 0.5;
% [ctrl_seq, reached, time] = find_opt_control(initial_state,value_funs, grid, dyn_sys, controls, uMode)

%% Helper functions
% TODO: Make x_ind, b_ind depend on real values not indices
function value_fun = construct_value_fun(x_ind, b_ind, gmin, gmax, gnums)
    grid = createGrid(gmin, gmax, gnums);
    target_set = ones(size(grid.xs{1}));
    target_set(x_ind(1):x_ind(2),b_ind(1):b_ind(2)) = -1;
    value_fun = compute_fmm_map(grid, target_set);
end

function [ctrl_seq, reached, time] = find_opt_control(initial_state,value_funs, grid, dyn_sys, controls, uMode)
    [~, num_timesteps] = size(value_funs);
    n = numel(initial_state);
    
    % Find latest time BRS includes initial state.
    initial_idx = grid.RealtoIdx(initial_state);
    initial_idx = initial_idx{1};
    start_t = 0;
    i = num_timesteps;
    while i > 0
        if value_funs{i}(initial_idx) <= 0
            start_t = i;
            break
        end
        i = i - 1;
    end
    
    if start_t == num_timesteps
        reached = true;
        time = 0;
    elseif start_t == 0
        reached = false;
        time = inf;
    else
        reached = true;
        ctrl_seq = cell(1,num_timesteps-start_t);
        state = grid.RealToCoords(initial_state);
        j = 1;
        time = 0;
        for t=start_t:num_timesteps-1
%             state
            vals = [];
            value_fun_next = value_funs{t+1};
            for i=1:numel(controls)
                u_i = controls(i);
                idx = grid.RealtoIdx(dyn_sys.dynamics(state,u_i));
                val = value_fun_next(idx{1});
                vals = [vals, val];
            end
            if strcmp(uMode, "min")
                [~, ctrl_ind] = min(vals, [], 2);
            else
                [~, ctrl_ind] = max(vals, [], 2);
            end
            ctrl = controls(ctrl_ind);
            ctrl_seq{j} = ctrl;
            state = grid.RealToCoords(dyn_sys.dynamics(state,ctrl));
            time = time + dyn_sys.dt;

            j = j + 1;
        end
    end
end
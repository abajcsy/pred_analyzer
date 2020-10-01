function [traj, traj_tau, ctrl_seq, reached, time] = ...
    computeOptTraj_old(initial_state, value_funs, grid, dyn_sys, controls, ...
    uMode, likelyMasks, start_idx, zero_tol)

    [~, num_timesteps] = size(value_funs);
    n = numel(initial_state);
    
    % Find latest time BRS includes initial state.
    initial_coord = grid.RealToCoords(initial_state);
    traj(1:3,1) = cell2mat(initial_coord);
    traj_tau(1) = 0.0;
    initial_idx = grid.RealToIdx(initial_state);
    initial_idx = initial_idx{1};
    start_t = 0;
    i = num_timesteps;
    
    while i >= start_idx
        fprintf('Value of initial state at t=-%f: %f ...\n', i, ...
                value_funs{i}(initial_idx));
        if value_funs{i}(initial_idx) <= zero_tol
            fprintf('Value of initial state at earliest appearance in BRS: %f\n', ...
                value_funs{i}(initial_idx));
            start_t = i;
            break
        end
        i = i - 1;
    end
    if start_t == num_timesteps
        reached = true;
        time = 0;
        ctrl_seq = [];
    elseif start_t == 0
        reached = false;
        time = inf;
        ctrl_seq = [];
    else
        reached = true;
        ctrl_seq = cell(1,num_timesteps-start_t);
        state = initial_coord;
        j = 1;
        time = 0;
        
        for t=start_t:num_timesteps-1
%             state
            vals = [];
            value_fun_next = value_funs{t+1};
            idx_curr = grid.RealToIdx(state);
            for i=1:numel(controls)
                u_i = controls{i};
                idx = grid.RealToIdx(dyn_sys.dynamics(state,u_i));
                likelyMask = likelyMasks(num2str(u_i));
                val = value_fun_next(idx{1}) * likelyMask(idx_curr{1});
                vals = [vals, val];
            end
            if strcmp(uMode, "min")
                [~, ctrl_ind] = min(vals, [], 2);
            else
                [~, ctrl_ind] = max(vals, [], 2);
            end
            ctrl = controls{ctrl_ind};
            ctrl_seq{j} = ctrl;
            state = grid.RealToCoords(dyn_sys.dynamics(state,ctrl));
            time = time + 1;
            
            traj(1:3, j+1) = cell2mat(state);
            traj_tau(j+1) = time;

            j = j + 1;
        end
    end
end
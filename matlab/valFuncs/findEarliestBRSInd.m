function tEarliest = findEarliestBRSInd(grid, value_funs, z, upper, lower)
% tEarliest = findEarliestBRSInd(grid, value_funs, z, upper, lower)
%     Determine the earliest time that the current state z is in the reachable set
% 
% Inputs:
%     grid, value_funs  - grid and value function representing reachable set
%     z                 - state of interest
%     upper, lower      - upper and lower time indices of the search range
%                         (e.g. upper = final time idx, lower = initial time idx)
%
% Output:
%     tEarliest - earliest time index that z is in the reachable set
%               - returns 0 if can't find.

initial_idx = grid.RealToIdx(z);
initial_idx = initial_idx{1};

zero_tol = 1e-4;
tEarliest = 0;
i = upper;

while i >= lower
    fprintf('Value of initial state at t=-%f: %f ...\n', i, ...
            value_funs{i}(initial_idx));
    if value_funs{i}(initial_idx) <= zero_tol
        fprintf('Value of initial state at earliest appearance in BRS: %f\n', ...
            value_funs{i}(initial_idx));
        tEarliest = i;
        break
    end
    i = i - 1;
end
end
function tEarliest = find_earliest_BRS_ind(g, data, x, upper, lower)
% tEarliest = find_earliest_BRS_ind(g, data, x, upper, lower)
%     Determine the earliest time that the current state is in the reachable set
% 
% Inputs:
%     g, data - grid and value function representing reachable set
%     x       - state of interest
%     upper, lower - upper and lower indices of the search range
%
% Output:
%     tEarliest - earliest time index that x is in the reachable set

if nargin < 4
  upper = size(data, g.dim+1);
  lower = 1;
end

% Binary search
clns = repmat({':'}, 1, g.dim);
small = 1e-4;

first_t_neg = nan;
last_t_neg = nan;

tidx = lower;
while tidx <= upper
  %tEarliest = ceil((upper + lower)/2);
  valueAtX = eval_u(g, data{tidx}(clns{:}), x);
  
  if valueAtX < small
      if isnan(first_t_neg)
          first_t_neg = tidx;
          last_t_neg = tidx;
      else
          last_t_neg = tidx;
      end
    % point is in reachable set; eliminate all lower indices
    %lower = tEarliest;
  %else
    % too late
   % upper = tEarliest - 1;
  end
  
  tidx = tidx + 1;
end

if isnan(first_t_neg) || isnan(last_t_neg)
  fprintf('Could not find earliest time where the state is in BRS!');
end

tEarliest = last_t_neg;
%tEarliest = upper;
end
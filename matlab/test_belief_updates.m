%% Goal locations (unknown).
g1 = -4;
g2 = 4;
goals = [g1, g2];

%% Time and probability discretization.
dt = 1;
dp = 0.1;

%% Number of discrete x-points and belief-points.
nx = abs(g1-g2)/dt + 1;
np = 1/dp + 1;

%% Set of controls.
ctrls = [0,1]; % 0 == LEFT, 1 == RIGHT.

%% Initial state.
x0 = 0.;
b0 = [0.5, 0.5];

%% Observe a sequence of RIGHT actions. 
u = 1;

%% dt = 1
bnext_dt1 = belief_update(u, x0, b0, goals, 1)

%% dt = 0.5
bnext1_dt05 = belief_update(u, x0, b0, goals, 0.5);
bnext1_dt1 = belief_update(u, x0, bnext1_dt05, goals, 0.5)

%% dt = 0.2
bnext2_dt02 = belief_update(u, x0, b0, goals, 0.2);
bnext2_dt04 = belief_update(u, x0, bnext2_dt02, goals, 0.2);
bnext2_dt06 = belief_update(u, x0, bnext2_dt04, goals, 0.2);
bnext2_dt08 = belief_update(u, x0, bnext2_dt06, goals, 0.2);
bnext2_dt1 = belief_update(u, x0, bnext2_dt08, goals, 0.2)

%% variable dt = 0.5, 0.1, 0.2, 0.2
bnext3_dt05 = belief_update(u, x0, b0, goals, 0.5);
bnext3_dt06 = belief_update(u, x0, bnext3_dt05, goals, 0.1);
bnext3_dt08 = belief_update(u, x0, bnext3_dt06, goals, 0.2);
bnext3_dt1 = belief_update(u, x0, bnext3_dt08, goals, 0.2)

%% Reachability setup!
% x = linspace(g1, g2, nx);
% bg = linspace(0,1, np);
% 
% [xgrid, bgrid] = meshgrid(x, bg);
% 
% t_hor = 3;
% 
% value_funs = cell(1, t_hor);
% times = linspace(0,t_hor, dt);
% 
% value_funs{end} = zeros(size(xgrid));

function bnext = belief_update(u, x, b, goals, dt)
    bnext = [0.,0.];
    normalizer = 0.;
    for i=1:length(goals)
        g = goals(i);
        bnext(i) = pugiveng(u, x, g, goals, dt) * b(i);
        normalizer = normalizer + bnext(i);
    end
    bnext = bnext / normalizer;
end

function pu = pugiveng(u, x, g, goals, dt)
    xnext = dynamics(x, u, dt);
    numerator = exp(-abs(xnext - g));
    denominator = exp(-abs(xnext - goals(1))) + exp(-abs(xnext - goals(2)));
    pu = numerator / denominator;
end

function xnext = dynamics(x, u, dt)
    if u == 0 % LEFT
        xnext = x - dt*1.;
    elseif u == 1 % RIGHT
        xnext = x + dt*1.;
    else
        error("Invalid control!");
    end
end
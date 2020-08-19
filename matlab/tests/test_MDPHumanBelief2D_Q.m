clear all
clf
close all

%% Grid setup
gmin = [-4, -4, 0];
gmax = [4, 4, 1];
gnums = [20, 20, 20];
g = createGrid(gmin, gmax, gnums);

%% Joint Dynamics Setup.
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;
gamma = 0.9;
eps = 0.01;
uThresh = 0.00;

% Obstacles for Q-function
obs_center = [-1; 1];
obs_width = [1; ...
          1;];
      
goal_val = 1;
obs_val = -10;

goal_rad = 1;
g_phys = createGrid([gmin(1), gmin(2)], [gmax(1), gmax(2)], [gnums(1), gnums(2)]);

v_inits = cell(1,numel(thetas));
for i=1:numel(v_inits)
    v_data = compute_v_data(g_phys, goal_val, thetas{i}, goal_rad, obs_val, obs_center, obs_width);
    v_grid = Grid([gmin(1), gmin(2)], [gmax(1), gmax(2)], [gnums(1), gnums(2)]);
    v_grid.SetData(v_data);
    v_inits{i} = v_grid;
    figure;
    s = surf(g_phys.xs{1}, g_phys.xs{2}, v_data);
    shading interp
end

% Initial state and dynamical system setup
initial_state = {0, 0, 0.1};

% MDP human.
gdisc = (gmax - gmin) ./ (gnums - 1);
dyn_sys = MDPHumanBelief2D_Q(initial_state, v_inits, trueThetaIdx, uThresh, gdisc, gamma, eps);

for i=1:numel(v_inits)
    figure;
    s = surf(g_phys.xs{1}, g_phys.xs{2}, dyn_sys.v_funs{i});
    shading interp
end

% TODO: Change goal_fun to circle (using shapeSphere).
% Assumes goal_value >= 0 and obs_value <= 0.
function v_data = compute_v_data(g, goal_value, goal_center, goal_rad, obs_value, obs_center, obs_width)
    goal_fun = shapeRectangleByCenter(g, ...
        [goal_center(1); goal_center(2)], [goal_rad; goal_rad]);
    
    goal_fun(goal_fun>=0) = 0;
    goal_fun(goal_fun<0) = goal_value;
    
    
    obs_fun = shapeRectangleByCenter(g, obs_center, obs_width);
    obs_fun(obs_fun<0) = obs_value;
    obs_fun(obs_fun>=0) = 0;
    
    v_data = goal_fun + obs_fun;
end
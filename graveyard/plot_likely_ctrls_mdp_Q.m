clear all
clf
close all

%% File to plot likely controls
% Note that plot is of controls, not of next state (which would be
% gridded).

%% Grid setup
gmin = [-4, -4, 0];
gmax = [4, 4, 1];
gnums = [20, 20, 20];
g = createGrid(gmin, gmax, gnums);

%% Target Set Setup
tol = 0.1;
centerPgoal1 = 0.9;
xyoffset = 0.1;
center = [0; 0; centerPgoal1];
widths = [(gmax(1) - gmin(1)) - xyoffset; ...
          (gmax(2) - gmin(2)) - xyoffset; 
          tol];
initial_value_fun = shapeRectangleByCenter(g, center, widths);

%% Time vector
t0 = 1;
num_timesteps = 15;
tau = t0:1:num_timesteps;  % timestep in discrete time is always 1

%% Problem Setup
uMode = "min"; % min or max
uThresh = 0.15;%0.13; %0.14; % 0.16;

%% Plotting?
plot = true;        % Visualize the BRS and the optimal trajectory?

%% Joint Dynamics Setup.
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;
gamma = 0.9;
eps = 0.01;

% Obstacles for Q-function
obs_center = [-1; 1];
obs_width = [1; ...
          1;];
      
goal_val = 1;
obs_val = -inf;

goal_rad = 0.5;
g_phys = createGrid([gmin(1), gmin(2)], [gmax(1), gmax(2)], [gnums(1), gnums(2)]);

v_inits = cell(1,numel(thetas));
for i=1:numel(v_inits)
    v_data = compute_v_data(g_phys, goal_val, thetas{i}, goal_rad, obs_val, obs_center, obs_width);
    v_grid = Grid([gmin(1), gmin(2)], [gmax(1), gmax(2)], [gnums(1), gnums(2)]);
    v_grid.SetData(v_data);
    v_inits{i} = v_grid;
end

% Initial state and dynamical system setup
initial_state = {0, 0, 0.1};

% MDP human.
gdisc = (gmax - gmin) ./ (gnums - 1);
dyn_sys = MDPHumanBelief2D_Q(initial_state, v_inits, trueThetaIdx, uThresh, gdisc, gamma, eps);
controls = dyn_sys.controls;

xs = linspace(gmin(1),gmax(1),gnums(1));
ys = linspace(gmin(2),gmax(2),gnums(2));
pb = 0.1;
true_theta = thetas{trueThetaIdx};

sz = 20;
% for k=1:numel(controls)
%     u = controls{k};
%     pu_true = dyn_sys.pugivenxtheta(u,{3,-3,0.5},dyn_sys.q_funs{trueThetaIdx});
% end
figure
for i=1:length(xs)
    x = xs(i);
    for j=1:length(ys)
        y = ys(j);
        state = {x,y,pb};
        for k=1:numel(controls)
            u = controls{k};
            pu_true = dyn_sys.pugivenxtheta(u,state,dyn_sys.q_funs{trueThetaIdx});
            if pu_true >= uThresh
                next_state = dyn_sys.dynamics(state,u);
                dx = next_state{1} - x;
                dy = next_state{2} - y;
                quiver(x,y,dx,dy,0.4);
                
                % if the control is "STOP", then plot a point.
                if u(1) == 0 && u(2) == 0
                    scatter(x,y,'k');
                end
                hold on
            end
        end
    end
end
for i=1:length(thetas)
    theta = thetas{i};
%     plot(theta(1),theta(2),'r.','MarkerSize',sz)
    scatter(theta(1),theta(2),50,20,'filled')
    hold on
end

% Plot obstacle
patch([-1.5 -1.5 -0.5 -0.5], [0.5 1.5 1.5 0.5],'r')
alpha(0.3)

title_str = strcat("Likely Controls under theta=(", ...
    num2str(true_theta(1)), ", ", num2str(true_theta(2)), ...
    ") and uThresh=", num2str(uThresh));
title(title_str);
xlabel("x")
ylabel("y")

function controls = generate_controls(gdisc)
    controls = cell(1,9);
    xs = {0, -1*gdisc(1), gdisc(1)};
    ys = {0, -1*gdisc(2), gdisc(2)};
    
    ind = 1;
    for i=1:numel(xs)
        for j=1:numel(ys)
            controls{ind} = [xs{i}, ys{j}];
            ind = ind + 1;
        end
    end
end


function controls = generate_controls_ext(gdisc)
    controls = cell(1,9);
    xs = {0, -1*gdisc(1), 1*gdisc(1), -2*gdisc(1), 2*gdisc(1)};
    ys = {0, -1*gdisc(2), 1*gdisc(2), -2*gdisc(2), 2*gdisc(2)};
    
    ind = 1;
    for i=1:numel(xs)
        for j=1:numel(ys)
            controls{ind} = [xs{i}, ys{j}];
            ind = ind + 1;
        end
    end
end

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
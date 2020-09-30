clear all
clf
close all

%% File to plot likely controls
% Note that plot is of controls, not of next state (which would be
% gridded).

%% Grid setup
params.gmin = [-4, -4, 0];
params.gmax = [4, 4, 1];
params.gnums = [25, 25, 25];
params.g = createGrid(params.gmin, params.gmax, params.gnums);
params.extraArgs.g = params.g;
params.bdims = {3}; % dimension(s) which contain the belief

%% Joint Dynamics Setup.
params.theta = [3,3];
params.betas = {0.1, 1}; % Note that first two betas are included in state
params.trueBetaIdx = 1;

%% Pack Reward Info

% 2D Grid for computing Q-function.
g_phys = createGrid(params.gmin(1:2)', ...
                    params.gmax(1:2)', ...
                    params.gnums(1:2)');
params.reward_info.g = g_phys; 

% Obstacles used in Q-function computation.
% Axis-aligned rectangular obstacle convention is:
%       [lower_x, lower_y, width, height]
% params.reward_info.obstacles = {[-1.5, 0.5, 1, 1],...
%                                 [-1.5, -2, 1, 2]};
params.reward_info.obstacles = {};

% Setup theta info (convert to cell of cells).                      
params.reward_info.theta = {params.theta(1), params.theta(2)};

%% Create the Human Dynamical System.
% Initial state and dynamical system setup
params.initial_state = {0,0,0.5};

% Params for Value Iteration. 
params.gamma = 0.99; 
params.eps = 0.01;

% Variance on likelihood model: 
%   beta = 0 --> uniform dist, 
%   beta = 1 --> default 
%   beta = inf --> dirac delta on opt action)
params.beta = params.betas{params.trueBetaIdx};

% MDP human.
gdisc4D = (params.gmax - params.gmin) ./ (params.gnums - 1);

% dt induced by discretization
params.vel = 0.6;
params.dt = gdisc4D(1)/params.vel;

% range of belief values
b_space = linspace(params.gmin(params.bdims{1}),params.gmax(params.bdims{1}),params.gnums(params.bdims{1}));
b_range = [b_space(2) b_space(numel(b_space)-1)];

params.uThresh = 0.125;
        
params.dyn_sys = MDPHumanConfidence3D(params.initial_state, ...
                                        params.reward_info, ...
                                        params.trueBetaIdx, ...
                                        params.uThresh, ...
                                        gdisc4D, ...
                                        params.gamma, ...
                                        params.eps, ...
                                        params.betas, ...
                                        b_range);
                                    
controls = params.dyn_sys.controls;

xs = linspace(params.gmin(1),params.gmax(1),params.gnums(1));
ys = linspace(params.gmin(2),params.gmax(2),params.gnums(2));
pb = 0.1;
true_beta = params.betas{params.trueBetaIdx};

uThresh = params.uThresh;

figure
for i=1:length(xs)
    x = xs(i);
    for j=1:length(ys)
        y = ys(j);
        state = {x,y,pb};
        for k=1:numel(controls)
            u = controls{k};
            pu_true = params.dyn_sys.pugivenxbeta(u,state,true_beta);
            if pu_true >= uThresh
                next_state = params.dyn_sys.dynamics(state,u);
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

sz = 20;
plot(params.theta(1),params.theta(2),'r.','MarkerSize',sz)
hold on

title_str = strcat("Likely Controls under theta=(", ...
    num2str(params.theta(1)), ", ", num2str(params.theta(2)), ...
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
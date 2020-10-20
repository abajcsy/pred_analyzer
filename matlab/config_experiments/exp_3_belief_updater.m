function belief_updater = exp_3_belief_updater(gmin, gmax, gnums, thetas, vel, dt)

% NOTE: These variables shouldn't matter for the use of this class.
trueThetaIdx = 1; 
uThresh = 0.25;
initial_state = {6, 1.83, pi, 0.5};

bdims = {4}; % dimension(s) which contain the belief

% range of belief values
b_space = linspace(gmin(bdims{1}),gmax(bdims{1}),gnums(bdims{1}));
b_range = [b_space(2) b_space(numel(b_space)-1)];

% 2D Grid for computing Q-function.
g_phys = createGrid(gmin(1:3)', ...
                    gmax(1:3)', ...
                    gnums(1:3)');
reward_info.g = g_phys; 

% Obstacles used in Q-function computation.
% Axis-aligned rectangular obstacle convention is:
%       [lower_x, lower_y, width, height]
reward_info.obstacles = {[-7.75, -7.75, 4.1, 4.1]...
                        [3.65, -7.75, 4.1, 4.1], ...
                        [-7.75, 3.65, 4.1, 4.1], ...
                        [3.65, 3.65, 4.1, 4.1]};                         
                     
% Setup theta info (convert to cell of cells).                      
reward_info.thetas = cell(1,numel(thetas));
for i=1:numel(thetas)
    reward_info.thetas{i} = {thetas{i}(1), ...
                            thetas{i}(2), ...
                            thetas{i}(3)};
end

% Params for Value Iteration. 
gamma = 0.98; 
eps = 0.01;

% Variance on likelihood model: 
%   beta = 0 --> uniform dist, 
%   beta = 1 --> default 
%   beta = inf --> dirac delta on opt action)
beta = 1;

% State space discretization
gdisc4D = (gmax - gmin) ./ (gnums - 1);

% MDP human.
belief_updater = CarHumanBelief4D(initial_state, ...
                                reward_info, ...
                                trueThetaIdx, ...
                                uThresh, ...
                                gdisc4D, ...
                                gnums, ...
                                gamma, ...
                                eps, ...
                                beta, ...
                                vel, ...
                                dt, ...
                                b_range);                                
end
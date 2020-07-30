clf
clear all

% Grid setup
gmin = [-4, -4, 0];
gmax = [4, 4, 1];
gnums = [19,19,19];

% Joint Dynamics Setup.
dt = 0.5;
num_ctrls = 20;
controls = linspace(0,2*pi,num_ctrls);
v = 1;

% Initial state and dynamical system setup
initial_state = cell(1,3);
initial_state{1} = 0;
initial_state{2} = 0;
initial_state{3} = 0.5;
thetas = {[-2, 2], [2, 2]};
trueThetaIdx = 1;
uThresh = 0.0;
dyn_sys = HumanBelief2D(dt, thetas, num_ctrls, controls, ...
            initial_state, v, uThresh, trueThetaIdx);
        
% Create compute grid.        
compute_grid = Grid(gmin, gmax, gnums);

figure(1);
hold on
% plot initial state
scatter(initial_state{1}, initial_state{2}, ...
    'markerfacecolor', 'k', 'markeredgecolor', 'k');

% Setup the locations of each of the eight nearby 
% states of the grid that a next state could map to.
resx = compute_grid.gdisc(1);
resy = compute_grid.gdisc(2);
corner_coords = {[resx, 0], [resx, resy], ...
                    [0, resy], [-resx, resy], ...
                    [-resx, 0], [-resx, -resy], ...
                    [0, -resy], [resx, -resy]};
% Setup colors for each of the edge states.
rco = [1, 0, 0, 1, 1, 0.8, 1, 0,];
gco = [0, 1, 0, 1, 0, 0.7, 0.5, 0.5];
bco = [0, 0, 1, 0, 1, 0.1, 0, 0.8];
eps = 0.01;

% Visualize each of the eight nearby 
% states of the grid that a next state could map to.
for ci = 1:length(corner_coords)
    scatter(corner_coords{ci}(1), corner_coords{ci}(2), 200, 'd',...
        'markerfacecolor', 'none', ...
        'markeredgecolor', [rco(ci), gco(ci), bco(ci)],....
        'LineWidth',1.5);
end

for ui = 1:length(controls)
    u = controls(ui);
    
    % Get the next continuous state by applying control at current state.
    znext = dyn_sys.dynamics(initial_state, u);
    
    % Find the coordinate on the grid that this next state is closest to.
    znext_coord = compute_grid.RealToCoords(znext);
   
    % Set the color of the next state based on which of the eight nearby 
    % states of the grid it maps onto.
    face_color = [0.6,0.6,0.6];
    for ci = 1:length(corner_coords)
        dist_to_edge = sqrt((znext_coord{1} - corner_coords{ci}(1))^2 + ...
                (znext_coord{2} - corner_coords{ci}(2))^2);
            
        if dist_to_edge < eps
            face_color = [rco(ci), gco(ci), bco(ci)];
        end
    end
    
    % Plot the continuous next state.
    scatter(znext{1}, znext{2}, ...
        'markerfacecolor', face_color, 'markeredgecolor', face_color);
    
    % Plot the coordinate on the grid that the next state is closest to.
    scatter(znext_coord{1}, znext_coord{2}, ...
        'markerfacecolor', 'none', 'markeredgecolor', face_color, 'LineWidth',1.5);
end

% Setup plotting and viewing limits.
box on
set(gcf,'color','w'); 

set(gca,'xtick',linspace(gmin(1),gmax(1),gnums(1)));
set(gca,'ytick',linspace(gmin(2),gmax(2),gnums(2)));
set(gca,'ztick',linspace(gmin(3),gmax(3),gnums(3)));
xticks([-resx, 0, resx]);
yticks([-resy, 0, resy]);
xlim([-resx-0.1, resx+0.1]);
ylim([-resy-0.1, resy+0.1]);
xlabel('x');
ylabel('y');
grid on;
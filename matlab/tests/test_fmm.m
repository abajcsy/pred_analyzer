%% This is a simple test file for running the fast marching method (FMM)
% for computing the signed distance to an arbitrarily-shaped set.

params.gmin = [-4, -4, 0];
params.gmax = [4, 4, 0.95];
params.gnums = [40, 40, 40];

g_phys = createGrid(params.gmin(1:2)', ...
                    params.gmax(1:2)', ...
                    params.gnums(1:2)');

% Obstacles (based on interpolated occupancy grid) used in Q-function computation.
repo = what('pred_analyzer');
data_path = strcat(repo.path, '/matlab/data/');
% map_name = 'map.png';
map_name = 'cluttered_map_doorway.png';
obs_data = imread(strcat(data_path, map_name));
n_phys = numel(params.gmin) - numel(params.bdims);
occupancy_map = get_obs_map(obs_data, ...
                            params.gmin(1:n_phys), ...
                            params.gmax(1:n_phys), ...
                            params.gnums(1:n_phys));
% Round occupancy map to 0 (free) or 1 (obs), turn it into made up of
% +1s (free) and -1s (obs), and compute signed distance function.
occupancy_map = occupancy_map.data;
occupancy_map(occupancy_map<=0) = 0;
occupancy_map = 1 - 2.*occupancy_map;
grid_phys = createGrid(params.gmin(1:n_phys), ...
                        params.gmax(1:n_phys), ...
                        params.gnums(1:n_phys));
signed_dist = compute_fmm_map(grid_phys, occupancy_map);

% Plot signed distance function, l(x). 
surf(g_phys.xs{1}, g_phys.xs{2}, signed_dist);
zlabel('$l(z)$', 'Interpreter', 'Latex');
xlabel('$x$', 'Interpreter', 'Latex');
ylabel('$b(\theta)$', 'Interpreter', 'Latex');
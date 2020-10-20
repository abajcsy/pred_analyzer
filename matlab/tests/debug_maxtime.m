clear all
close all

%% Load data.
%load('exp_1_max_uthresh_0.15_theta_2_coarse.mat')
%load('exp_1_max_uthresh_0.15_theta_2.mat')
%load('exp_1_max_uthresh_0.15_theta_2_HIGH_RES.mat')

%load('exp_1_max_uthresh_0.15_theta_2_FIXED.mat')
%load('exp_1_min_uthresh_0.15_theta_2_FIXED.mat')

%load('exp_1_max_uthresh_0.15_theta_2_FIXED_LOW_RES.mat')
load('exp_1_max_uthresh_0.15_theta_2_FIXED_HIGH_RES.mat')

%% Create grid structure for linear indexing. 
grid_struct = Grid(params.g.min, params.g.max, params.g.N); % for converting from real to linear index

upper = params.tau(end);
lower = 1;

% Grid initial condition
z0_real = params.initial_state;
z0_coords = grid_struct.RealToCoords(z0_real);

%% Determine the earliest time (index) that the current state is in the reachable set
tEarliest = findEarliestBRSInd(grid_struct, value_funs, z0_coords, upper, lower);

ttr_idx = params.tau(end) - tEarliest + 1;
ttr_real = (ttr_idx-1)*params.dt;
fprintf('TTR: %f s \n', ttr_real);

z0_linidx = grid_struct.CoordsToIdx(z0_coords);
z0_linidx = z0_linidx{1};
vz = value_funs{tEarliest}(z0_linidx);

% Grab all the controls. 
all_ctrls = extraOuts.all_opt_ctrl_idxs;

v2d = VideoWriter('twoD.avi','Motion JPEG AVI');
v2d.Quality = 95;
v2d.open();

v3d = VideoWriter('threeD.avi','Motion JPEG AVI');
v3d.Quality = 95;
v3d.open();

%% Visualize the BRT slice at the current b(theta) dimension and in x-y. 
z = z0_coords;
for t=tEarliest:params.tau(end)-1
    [gOut, dataOut] = proj(params.g, value_funs{t}, [0,0,1], z{3});
    figure(1)
    hold on;
    h2d = visSetIm(gOut, dataOut);
    hp = scatter(z{1}, z{2}, 'k', 'filled');
    state_txt = strcat('z = [', num2str(z{1}), ',',num2str(z{2}), ',',num2str(z{3}),']');
    t1 = text(0,5,state_txt);
    t2 = text(0,4,strcat('TTE: ',num2str(ttr_real), ' s, num tsteps: ', num2str(ttr_idx-1)));
    xlabel('x')
    ylabel('y')
    set(gcf, 'color', 'w')
    set(gcf, 'position', [0,0,600,600])
    box on
    hold off
    frame = getframe(gcf);
    writeVideo(v2d,frame);
    
    figure(2)
    hold on;
    h3d = visSetIm(params.g, value_funs{t});
    h3d.FaceAlpha = 0.5;
    hp3d = scatter3(z{1}, z{2}, z{3}, 'k', 'filled');
    xlabel('x')
    ylabel('y')
    zlabel('b(g=g1)')
    view(-138, 29)
    set(gcf, 'color', 'w')
    set(gcf, 'position', [0,0,600,600])
    grid on
    box on
    hold off
    frame = getframe(gcf);
    writeVideo(v3d,frame);

    z_linidx = grid_struct.CoordsToIdx(z);
    z_linidx = z_linidx{1};
    
    curr_ctrl = all_ctrls{t};
    uopt_idx = curr_ctrl(z_linidx);
    
    % Get the optimal control
%     vz_vals = [];
%     next_value_fun = value_funs{t+1};
%     for ui=1:length(params.dyn_sys.controls)
%         u_candidate = params.dyn_sys.controls{ui};
%         znext = params.dyn_sys.dynamics(z, u_candidate);
%         znext_linidx = grid_struct.CoordsToIdx(znext);
%         znext_linidx = znext_linidx{1};
%         vz_vals = [vz_vals, next_value_fun(znext_linidx)];
%     end
%     [max_val, uopt_idx] = max(vz_vals);
%     all_uopt_idxs = find(vz_vals == max_val);
%     uopt_idx = all_uopt_idxs(1);
    
    
    uopt = params.dyn_sys.controls{uopt_idx}; 
    %uopt = ctrls{t-tEarliest+1};
    
    % Propagate dynamics. 
    z = params.dyn_sys.dynamics(z, uopt);
    pause(0.1);
    
    % delete old info. 
    h2d.LineColor = [0.8,0.8,0.8];
    hp.CData = [0.8,0.8,0.8];
    hp3d.CData = [0.8,0.8,0.8];
    
    delete(h3d);
    delete(t1);
    delete(t2);
end

[gOut, dataOut] = proj(params.g, value_funs{t}, [0,0,1], z{3});
figure(1)
hold on;
h2d = visSetIm(gOut, dataOut);
hp = scatter(z{1}, z{2}, 'k', 'filled');
state_txt = strcat('z = [', num2str(z{1}), ',',num2str(z{2}), ',',num2str(z{3}),']');
t1 = text(0,5,state_txt);
xlabel('x')
ylabel('y')
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])
box on
hold off
frame = getframe(gcf);
writeVideo(v2d,frame);

figure(2)
hold on;
h3d = visSetIm(params.g, value_funs{t});
h3d.FaceAlpha = 0.5;
hp3d = scatter3(z{1}, z{2}, z{3}, 'k', 'filled');
xlabel('x')
ylabel('y')
zlabel('b(g=g1)')
view(-138, 29)
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])
grid on
box on
hold off
frame = getframe(gcf);
writeVideo(v3d,frame);

v2d.close();
v3d.close();
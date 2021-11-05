clear all
close all

load('conf_full_sim.mat')

%% Video creation!
save_video = true;
if save_video 
    curr_date = datestr(now,'mm_dd_yyyy_HH_MM');
    filename = strcat('crosswalk_conf',curr_date,'.mp4');
    vout = VideoWriter(filename,'MPEG-4');
    vout.Quality = 100;
    vout.FrameRate = 5;
    vout.open;
end

%% PLOT!
figure(1)
hold on
% Plot environment.
bandw_cmap = [0,0,0;1,1,1]; %[1,1,1;0,0,0];
colormap(bandw_cmap)

% Plot obstacles.
for oi=1:length(robot_params.obstacles)
    obs_info = robot_params.obstacles{oi};
    obs_min = obs_info(1:2);

    x_min = obs_min(1);
    y_min = obs_min(2);
    rectangle('position', obs_info, ...
                'curvature', [0.1,0.1], ...
                'EdgeColor', 'k', ...
                'FaceColor', 'k');
end

% plot robot goal.
qrg = quiver(robot_params.goal(1), robot_params.goal(2), ...
    cos(robot_params.goal(3)), sin(robot_params.goal(3)), 'b');
qrg.Marker = 'o';
qrg.MarkerFaceColor = 'b';

% plot human goal.
scatter(human_params.goal(1), human_params.goal(2), 'r');
xlim([human_params.gmin(1), human_params.gmax(1)])
ylim([human_params.gmin(2), human_params.gmax(2)])
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])

%% Robot base info.
footprint_rad = robot_params.car_rad;

%% Setup handles. 
gray_c = [0.8,0.8,0.8];

rph = [];
tb1 = [];
tb2 = [];
all_contour_h = [];
rrecth = [];
rps_h = [];
rrecth = [];

%% RUN SIM.
for t=1:simT
    zt = traj(:,t);
    h_xcurr = zt(1:2)'; %all_h_states{t};
    r_xcurr = all_r_states{t};
    robot_plan = all_plans{t};
    human_preds = all_preds{t};
    pbeta = all_beliefs{t};
    
    hsh.CData = gray_c;
    rsh.Color = gray_c;
    rsh.MarkerFaceColor = 'w';
    if ~isempty(rrecth)
        rrecth.EdgeColor = gray_c;
        rrecth.FaceColor = 'w';
    end
    if ~isempty(rps_h)
        rps_h.CData = gray_c;
        rps_h.MarkerEdgeColor = gray_c;
    end
    
    
    %% Plot the predictions.
    if ~isempty(all_contour_h)
        for i=1:length(all_contour_h)
            delete(all_contour_h(i))
        end
    end
    flat_preds = cell2mat(human_preds);
    likely_flat_preds = flat_preds(flat_preds > 0.05);
    min_prob = min(likely_flat_preds(:));
    max_prob = max(likely_flat_preds(:));
    offset = 0.3429/2.;
    for i=length(human_preds):-1:1
        levels = [-0.1,0.1];
        preds = human_preds{i};
        threshold = 0.05;
        eps = robot_params.planner.compute_likely_states(human_preds{i}, threshold);
        locs = find(human_preds{i} >= eps);
        Xs = human_params.pred_g.xs{1}(locs); 
        Ys = human_params.pred_g.xs{2}(locs); 
        if eps == 0.0
            preds = (human_preds{i} > eps) .* 1.0 + (human_preds{i} <= eps) .* 0.0;
            locs = find(human_preds{i} > eps);
            Xs = human_params.pred_g.xs{1}(locs); 
            Ys = human_params.pred_g.xs{2}(locs); 
        end
        
        alpha = (preds(locs) - min_prob)/(max_prob - min_prob);
        colorID = zeros(length(locs),3);    
        colorID(:,1) = alpha*1 + (1-alpha)*0.35;
        colorID(:,2) = alpha*0 + (1-alpha)*0.9; 
        colorID(:,3) = alpha*0.46 + (1-alpha)*1; 
        
        h = scatter(Xs(:), Ys(:), 170, colorID, 'filled', 'marker', 's');
 
        all_contour_h = [all_contour_h, h];
    end

    %% Plot robot plan
    % Plot robot shared plan, and contingency plans.
    rps_h = scatter(robot_plan{1}, robot_plan{2}, 'markeredgecolor', 'b');

    % Update human state.
    %h_ctrl = pi/4;
    zt = traj(:,t+1);
    h_xnext = zt(1:2)';
    
    % Update robot state.
    ntsteps = floor(human_params.dt/robot_params.dt);
    r_xnext = all_r_states{t+1}; %[robot_plan{1}(r_tstep), robot_plan{2}(r_tstep), ...
              %              robot_plan{5}(r_tstep), robot_plan{3}(r_tstep)];
    
    %% Plot human and robot state.
    hsh = scatter(h_xcurr(1), h_xcurr(2), 'r', 'filled');
    
    % plot robot state body. 
    pos = [r_xcurr(1)-footprint_rad, ...
           r_xcurr(2)-footprint_rad, ...
           footprint_rad*2, footprint_rad*2];
    rrecth = rectangle('position', pos, ...
                'curvature', [1,1], ...
                'EdgeColor', 'b');
    rrecth.FaceColor = [153, 192, 255]/255.;
    % plot robot state center
    %rsh = scatter(r_xcurr(1), r_xcurr(2), 'k', 'markerfacecolor', 'w', 'markeredgecolor', 'k');
    rsh = quiver(r_xcurr(1), r_xcurr(2), ...
                cos(r_xcurr(3)), sin(r_xcurr(3)), 'b');
    rsh.Marker = 'o';
    rsh.MarkerFaceColor = 'b';
    
    %tatreplan = scatter(robot_plan{1}(ntsteps), robot_plan{2}(ntsteps), 'r', 'filled');
    
    % Plot the belief.
    if ~isempty(tb1)
        delete(tb1)
        delete(tb2)
    end
    tb1 = text(-5.5,4,strcat('b_0(low conf) = ', num2str(pbeta(1))));
    tb1.Color = 'r';
    tb1.FontSize = 12;
    tb2 = text(-5.5,4.5,strcat('b_0(high conf) = ', num2str(pbeta(2))));
    tb2.Color = 'r';
    tb2.FontSize = 12;
    
    if save_video
        current_frame = getframe(gcf); %gca does just the plot
        writeVideo(vout,current_frame);
    end
    pause(0.1)
end

if save_video
    vout.close()
end
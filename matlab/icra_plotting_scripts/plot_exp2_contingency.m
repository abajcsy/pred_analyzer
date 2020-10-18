clear all
close all

load('final_contingency_example_1090_CONSERVATIVE.mat');
load('exp_2_opt_human_traj_1090.mat');

%% Video creation!
save_video = true;
if save_video 
    curr_date = datestr(now,'mm_dd_yyyy_HH_MM');
    filename = strcat('exp2_contingency_',curr_date,'.mp4');
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
%ph = pcolor(params.g2d.xs{1}, params.g2d.xs{2}, params.sd_obs);
ph = pcolor(robot_params.gimg.xs{1}, robot_params.gimg.xs{2}, robot_params.obs_map_full);
set(ph, 'EdgeColor', 'none');
% plot robot goal.
qrg = quiver(robot_params.goal(1), robot_params.goal(2), ...
    cos(robot_params.goal(3)), sin(robot_params.goal(3)), 'b');
qrg.Marker = 'o';
qrg.MarkerFaceColor = 'b';
% plot human goal.
scatter(opt_human_params.goal(1), opt_human_params.goal(2), 'r');
xlim([opt_human_params.gmin(1), opt_human_params.gmax(1)])
ylim([opt_human_params.gmin(2), opt_human_params.gmax(2)])
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])

%% Robot base info.
pr2_base_width = 0.68; % m
footprint_rad = pr2_base_width/2.;

%% Setup handles. 
gray_c = [0.8,0.8,0.8];

rph = [];
tb1 = [];
tb2 = [];
all_contour_h = [];
rrecth = [];
rps_h = [];
rpopt_h = [];
rpfrs_h = [];
rrecth = [];

%% RUN SIM.
for t=1:simT
    zt = traj(:,t);
    h_xcurr = zt(1:2)'; %all_h_states{t};
    r_xcurr = all_r_states{t};
    robot_plan = all_plans{t};
    opt_human_preds = all_opt_preds{t};
    frs_human_preds = all_frs_preds{t};
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
        rpopt_h.CData = gray_c;
        rpopt_h.MarkerEdgeColor = gray_c;
        rpfrs_h.CData = gray_c;
        rpfrs_h.MarkerEdgeColor = gray_c;
    end
    
%     %% Plot the predictions.
%     r_color = linspace(0.1,0.9,length(human_preds));
%     if ~isempty(all_contour_h)
%         for i=1:length(all_contour_h)
%             delete(all_contour_h(i))
%         end
%     end
%     for i=length(human_preds):-1:1
%         levels = [-0.1,0.1];
%         preds = human_preds{i};
%         threshold = robot_params.pthresh;
%         eps = robot_params.planner.compute_likely_states(human_preds{i}, threshold);
%         locs = find(human_preds{i} > eps);
%         Xs = human_params.pred_g.xs{1}(locs); 
%         Ys = human_params.pred_g.xs{2}(locs); 
%         if eps == 0.0
%             preds = (human_preds{i} > eps) .* 1.0 + (human_preds{i} <= eps) .* 0.0;
%             locs = find(human_preds{i} > eps);
%             Xs = human_params.pred_g.xs{1}(locs); 
%             Ys = human_params.pred_g.xs{2}(locs); 
%         end
%         h = scatter(Xs(:), Ys(:), 'r', 'markeredgecolor', [1,r_color(i),r_color(i)]);
%         all_contour_h = [all_contour_h, h];
%     end

    %% Plot robot plan
    % Plot robot shared plan, and contingency plans.
    rps_h = scatter(shared_plan{1}, shared_plan{2}, 'markeredgecolor', 'k');
    rpopt_h = scatter(opt_plan{1}, opt_plan{2}, 'markeredgecolor', 'r', 'markerfacecolor', 'r');
    rpfrs_h = scatter(frs_plan{1}, frs_plan{2}, 'markeredgecolor', 'c');

    % Update human state.
    %h_ctrl = pi/4;
    zt = traj(:,t+1);
    h_xnext = zt(1:2)';%[h_xcurr(1) + opt_human_params.dt * cos(h_ctrl), ...
              % h_xcurr(2) + opt_human_params.dt * sin(h_ctrl)];
    
    % Update robot state.
    ntsteps = floor(opt_human_params.dt/robot_params.dt);
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
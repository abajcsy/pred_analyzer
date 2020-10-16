clear all
close all

%% Load up all the info for robot.
robot_params = exp2_planner_baselines();

%% Load up all the info for the human.
% Setup what kind of collision checking we will do
coll_check = 'conf'; 
human_params = exp2_conf_pred();

%% Video creation!
save_video = false;
if save_video 
    curr_date = datestr(now,'mm_dd_yyyy_HH_MM');
    filename = strcat('exp2_',coll_check,'_baseline_',curr_date,'.mp4');
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
    cos(robot_params.goal(3)), sin(robot_params.goal(3)), 'k');
qrg.Marker = 'o';
qrg.MarkerFaceColor = 'k';
% plot human goal.
scatter(human_params.goal(1), human_params.goal(2), 'r');
xlim([human_params.gmin(1), human_params.gmax(1)])
ylim([human_params.gmin(2), human_params.gmax(2)])
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])

%% Setup robot start state. 
r_start = [2, 3, -pi/4, 0.01]; %[1, 4, -pi/4, 0.01];
rsh = scatter(r_start(1), r_start(2), 'k', 'filled');

%% Setup human start state. 
h_start = [1.4, 1.7]; %[2.2, 0.6]; %[2.6, 1];
% plot human state
hsh = scatter(h_start(1), h_start(2), 'r', 'filled');

%% Setup simulation horizon. 
simT = 10;
h_xcurr = h_start;
r_xcurr = r_start;

%% Setup prior over betas.
pbeta = human_params.beta_prior; 

%% Setup handles. 
gray_c = [0.8,0.8,0.8];

rph = [];
tb1 = [];
tb2 = [];
all_contour_h = [];

for t=1:simT
    % Predict human!
    fprintf('Predicting...\n');
    human_preds = ...
        human_params.predictor.predict(h_xcurr, human_params.T, pbeta);
    
    % Plan for the robot!
    fprintf('Planning...\n');
    robot_plan = ...
        robot_params.planner.plan(r_xcurr, robot_params.goal, ...
            human_preds, human_params.real_times, human_params.pred_g, coll_check);
    
    % Update human state.
    h_ctrl = pi/4;
    h_xnext = [h_xcurr(1) + human_params.dt * cos(h_ctrl), ...
               h_xcurr(2) + human_params.dt * sin(h_ctrl)];
    
    % Update robot state.
    r_xnext = [robot_plan{1}(2), robot_plan{2}(2), robot_plan{5}(2), robot_plan{3}(2)];
    
    %% Plot the predictions.
    r_color = linspace(0.1,0.9,length(human_preds));
    if ~isempty(all_contour_h)
        for i=1:length(all_contour_h)
            delete(all_contour_h(i))
        end
    end
    for i=1:length(human_preds)
        levels = [-0.1,0.1];
        preds = human_preds{i};
        if strcmp(coll_check, 'conf')
            eps = robot_params.planner.compute_likely_states(human_preds{i}, robot_params.pthresh);
            levels = [-0.1,eps];
            if eps == 0.0
                preds = (human_preds{i} > eps) .* 1.0 + (human_preds{i} <= eps) .* 0.0;
                levels = [1,1];
            end
        end
        
        [~,h] = contour(human_params.pred_g.xs{1}, human_params.pred_g.xs{2}, preds, ...
            levels, 'Color', [1,r_color(i),r_color(i)]);
        all_contour_h = [all_contour_h, h];
    end
    
    %% Plot robot plan
    hsh.CData = gray_c;
    rsh.CData = gray_c;
    if ~isempty(rph)
        rph.CData = gray_c;
    end
    rph = scatter(robot_plan{1}, robot_plan{2}, 'k');
    hsh = scatter(h_xcurr(1), h_xcurr(2), 'r', 'filled');
    rsh = scatter(r_xcurr(1), r_xcurr(2), 'k', 'filled');
    
    % Plot the belief.
    if strcmp(coll_check, 'conf')
        if ~isempty(tb1)
            delete(tb1)
            delete(tb2)
        end
        tb1 = text(-4.7,-1,strcat('b_{', num2str(t-1),'}(low conf) = ', num2str(pbeta(1))));
        tb1.Color = 'r';
        tb2 = text(-4.7,-1.5,strcat('b_{', num2str(t-1),'}(high conf) = ', num2str(pbeta(2))));
        tb2.Color = 'r';
    end
    
    if save_video
        current_frame = getframe(gcf); %gca does just the plot
        writeVideo(vout,current_frame);
    end
    pause(0.1)
    
    %% Update belief over betas.
    pbeta = human_params.predictor.belief_update(h_xnext, h_xcurr, pbeta);

    
    %% Finally update states.
    h_xcurr = h_xnext; 
    r_xcurr = r_xnext;
end

if save_video
    vout.close()
end
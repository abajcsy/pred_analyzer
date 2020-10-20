%% File to plot likely controls
clear all
clf
close all

%% Load all the parameters for this computation!
%params = mdpHuman3DSimpleEnv();
params = exp1_legibility();
xs = linspace(params.gmin(1),params.gmax(1),params.gnums(1));
ys = linspace(params.gmin(2),params.gmax(2),params.gnums(2));

true_theta = params.thetas{params.trueThetaIdx};
pb = 0.1; % doesn't matter for this plotting. 
sz = 20;

figure
hold on

%% Plot the environment
% plot obstacle.
bandw_cmap = [1,1,1;0,0,0]; 
colormap(bandw_cmap)
ph = pcolor(params.gimg.xs{1}, params.gimg.xs{2}, params.obs_map_full);
set(ph, 'EdgeColor', 'none');

%% Plot the sufficiently-likely controls at each state. 
endColorRed = [255., 0., 0.]/255.;
endColorBlue = [38., 138., 240.]/255.;

if params.trueThetaIdx == 1
    color = endColorRed;
else
    color = endColorBlue;
end
for i=1:length(xs)
    x = xs(i);
    for j=1:length(ys)
        y = ys(j);
        state = {x,y,pb};
        %color = [rand,rand,rand];
        for uidx=1:length(params.dyn_sys.controls)
            ucell = params.dyn_sys.controls(uidx);
            u = ucell{1};
            q_fun = params.dyn_sys.q_funs{params.trueThetaIdx};
            pu_true = params.dyn_sys.pugivenxtheta(u,state,q_fun);
            if pu_true > params.uThresh
                next_state = params.dyn_sys.dynamics(state,u);
                dx = next_state{1} - x;
                dy = next_state{2} - y;
                quiver(x,y,dx,dy,0.4,'Color', color);
                
                % if the control is "STOP", then plot a point.
                if u(1) == 0 && u(2) == 0
                    scatter(x,y,sz,color);
                end
            end
        end
    end
end

%% Plot the goals.
for i=1:length(params.thetas)
    theta = params.thetas{i};
    plot(theta(1),theta(2),'r.','MarkerSize',sz)
end

%% Ajust figure info.
xlim([params.gmin(1),params.gmax(1)])
ylim([params.gmin(2),params.gmax(2)])
xlabel("x")
ylabel("y")
set(gcf, 'color', 'w')
set(gcf, 'position', [0,0,600,600])
box on
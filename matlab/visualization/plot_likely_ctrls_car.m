%% File to plot likely controls
clear all
clf
close all

%% Load all the parameters for this computation!
params = carHuman4DDrivingEnv();
xs = linspace(params.gmin(1),params.gmax(1),params.gnums(1));
ys = linspace(params.gmin(2),params.gmax(2),params.gnums(2));
phis = linspace(params.gmin(3),params.gmax(3),params.gnums(3));

true_theta = params.thetas{params.trueThetaIdx};
pb = 0.1; % doesn't matter for this plotting. 
sz = 20;

%% Plot the sufficiently-likely controls at each state. 
figure
hold on

ang_vel = (2*pi/params.gnums(3))/params.dt;

trueTheta = 1;
state = {5.6, 2.25, pi, 0.5};
next_state1 = {4.6202, 2.2500, 3.1416, 0.6344};
next_state2 = {4.6362, 2.0974, 3.4558, 0.2631};

pus = plot_likely_us(state, params, 'k', trueTheta);
pus1 = plot_likely_us(next_state1, params, 'r', trueTheta);
pus2 = plot_likely_us(next_state2, params, 'b', trueTheta);

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

function pus = plot_likely_us(state, params, color, trueTheta)
    pus = [];
    for uidx=1:length(params.dyn_sys.controls)
        % u \in {'STOP', 'FORWARD', 'FORWARD_CW1', 'FORWARD_CCW1'}
        ucell = params.dyn_sys.controls(uidx);
        u = ucell{1};
        q_fun = params.dyn_sys.q_funs{trueTheta};
        pu_true = params.dyn_sys.pugivenxtheta(u,state,q_fun);
        pus = [pus, pu_true];
        if pu_true > params.uThresh
            %scatter(x,y,sz,color);
            next_state = params.dyn_sys.dynamics(state,u);
            dx = next_state{1} - state{1};
            dy = next_state{2} - state{2};
            quiver(state{1},state{2},dx,dy,0.4,'Color', color);

            % if the control is "STOP", then plot a point.
            if u == 1
                scatter(state{1},state{2},sz,color);
            end
        end
    end
end

% for k=1:2:length(phis)
%     figure
%     hold on
%     phi = phis(k);
%     title(strcat('P(u | (x,y,', num2str(phi), ') , g1) > ', num2str(params.uThresh)));
%     for i=1:length(xs)
%         x = xs(i);
%         for j=1:length(ys)
%             y = ys(j);
%             state = {x,y,phi,pb};
%             color = [rand,rand,rand];
%             
%             pus = [];
%             for uidx=1:length(params.dyn_sys.controls)
%                 % u \in {'STOP', 'FORWARD', 'FORWARD_CW1', 'FORWARD_CCW1'}
%                 ucell = params.dyn_sys.controls(uidx);
%                 u = ucell{1};
%                 q_fun = params.dyn_sys.q_funs{params.trueThetaIdx};
%                 pu_true = params.dyn_sys.pugivenxtheta(u,state,q_fun);
%                 pus = [pus, pu_true];
%                 if pu_true > params.uThresh
%                     %scatter(x,y,sz,color);
%                     next_state = params.dyn_sys.dynamics(state,u);
%                     dx = next_state{1} - x;
%                     dy = next_state{2} - y;
%                     quiver(x,y,dx,dy,0.4,'Color', color);
% 
%                     % if the control is "STOP", then plot a point.
%                     if u == 1
%                         scatter(x,y,sz,color);
%                     end
%                 end
%             end
%             pus
% %             pus
%         end
%     end
% end
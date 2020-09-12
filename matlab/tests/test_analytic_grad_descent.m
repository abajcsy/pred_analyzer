%% Tests how analytic stochastic gradient descent works for goals.
%           Q(x,u; theta) = -||x + f(x,u) - \theta||^2_2
%               x       \ in R^2
%               u       \in R
%               \theta  \in R^2
%  SGD Update:
%   theta_n+1 = theta_n + \alpha \nabla_\theta Q(x,u; theta)
clear all
clf

%% Parameters of SGD and dynamics.
alpha = 0.1;
v = 0.6;
dt = 0.3;

%% Initialize SGD parameter and initial human state
theta0 = [-3; -4];
x0 = [0; 0];

%% Iterate!
theta = theta0;
x = x0;
u = pi/2;

numT = 60;
tcolor = linspace(0.6,0, numT);
xcolor = linspace(0.6,0, numT);

hold on
for t=1:numT
    scatter(x(1), x(2), 'filled', 'markerfacecolor', ones(1,3)*xcolor(t));
    scatter(theta(1), theta(2), 'markeredgecolor', [1,tcolor(t),tcolor(t)]);
    xlim([-5,5]);
    ylim([-5,5]);
    box on
    
    if t < 10 %25
        u = pi/2;
    elseif t < 20 % 50
        u = pi/4;
    elseif t < 30 %75 
        u = -pi/4;
    else
        u = -pi/2;
    end
    
    theta = sgd(alpha, x, u, theta, v, dt);
    x = [x(1) + dt * v * cos(u); 
         x(2) + dt * v * sin(u)];
end

%% Perform SGD given current state/action and theta estimate. 
%     theta_n+1 = theta_n + \alpha \nabla_\theta Q(x,u; theta)
function theta_n = sgd(alpha, x, u, theta, v, dt)
    grad = Q_grad(x, u, theta, v, dt);
    theta_n = theta + alpha * grad;
end

%% Returns gradient of Q-function wrt \theta:
%   \nabla_\theta Q(x,u; theta) evaluated at current theta_n
function grad = Q_grad(x, u, theta, v, dt)
    grad = [2*(x(1) + dt * v * cos(u) - theta(1));
            2*(x(2) + dt * v * sin(u) - theta(2))];
end
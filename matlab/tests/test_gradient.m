%% Grid setup
gmin = [-4, -4, -4, -4];
gmax = [4, 4, 4, 4];
gnums = [20, 20, 2, 2];
g = createGrid(gmin, gmax, gnums);
gdisc4D = (gmax - gmin) ./ (gnums - 1);

thetas = {[-4,-4], [-4,4], [4,-4], [4,4]};
g2D = createGrid(gmin(1:2), gmax(1:2), gnums(1:2));

%% Gen ctrls
controls = generate_controls_mdp(gdisc4D);

%% Compute all Q-funs
all_q_funs = zeros([g.N(1), ...
                    g.N(2), ...
                    length(controls), ...
                    4]);
for ti=1:length(thetas)
    theta = thetas{ti};
    for ui=1:length(controls)
        u = controls{ui};
        q_fun = compute_q_fun(g2D.xs, u, theta);
        all_q_funs(:,:,ui,ti) = q_fun;
    end
end

%% Compute gradient!
low = [g.min(1), g.min(2), 1, g.min(3)];
up = [g.max(1), g.max(2), length(controls), g.max(3)];
N = [g.N(1), g.N(2), length(controls), 4];
deriv_grid = createGrid(low,up,N);

deriv_dim = 4;
generateAll = false;

[derivQ_L, derivQ_R] = ...
         upwindFirstFirst(deriv_grid, all_q_funs, deriv_dim, generateAll);
     
dQ_numeric = 0.5*(derivQ_L + derivQ_R);

dQ_analytic = zeros(size(20,20,9,4));
for ti=1:length(thetas)
    theta = thetas{ti};
    for ui=1:length(controls)
        u = controls{ui};
        dQ_tmp = analytic_dQ(g2D.xs, u, theta);
        dQ_analytic(:,:,ui,ti) = dQ_tmp;
    end
end
    
dQ_1 = squeeze(dQ_numeric(:,:,1,1));
dQ_2 = squeeze(dQ_numeric(:,:,1,2));

surf(g2D.xs{1},g2D.xs{2}, dQ_1)

function dQ = analytic_dQ(x,u,theta)
    xnext = physical_dynamics(x,u);
    next_pts = [xnext{1}(:), xnext{2}(:)];
    theta_pts = repmat(theta, length(next_pts), 1);
    dq_pts = 2 * (next_pts - theta_pts);
    
    dQ_1 = 2 .* (xnext{1} - theta(1));
    dQ_2 = 2 .* (xnext{2} - theta(2));
    
end

function q_fun = compute_q_fun(x,u,theta) 
    xnext = physical_dynamics(x,u);
    
    % Make next states into a matrix. 

    % Get phi(x,u) at the next state after applying u
%     phi_g = eval_u(obj.reward_info.g, obj.goal_feature, next_points);

    q_fun = -((xnext{1} - theta(1)).^2 + (xnext{2} - theta(2)).^2).^0.5;
end

%% Dyanmics of physical states.
function znext = physical_dynamics(z,u)
    % Return next state after applying control u to z. 
    % Note that no interpolation is done (should be handled by Grid class).
    znext = cell(2);
    znext{1} = z{1} + u(1);
    znext{2} = z{2} + u(2);
end


%% Generates the discrete controls for the MDP human. 
function controls = generate_controls_mdp(gdisc)
    nc = 9;
    controls = cell(1,nc);
    xs = {0, -1*gdisc(1), gdisc(1)};
    ys = {0, -1*gdisc(2), gdisc(2)};

    ind = 1;
    for i=1:numel(xs)
        for j=1:numel(ys)
            controls{ind} = [xs{i}, ys{j}];
            ind = ind + 1;
        end
    end
end
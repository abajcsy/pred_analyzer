function grid_obs = get_obs_map(A, gmin, gmax, sz)
    % Accepts A where 0 denotes obstacles, and returns the opposite as a Grid structure.
    A = 1 - double(A);
    
    m = sz(1);
    n = sz(2);
    
    gnums = size(A);
    g = createGrid(gmin, gmax, gnums);

    pts = [];
    xs = linspace(gmin(1),gmax(1),m);
    ys = linspace(gmin(2),gmax(2),n);
    for i=1:numel(xs)
        for j=1:numel(ys)
            pts = [pts ; xs(i) ys(j)];
        end
    end

    v = eval_u(g, A, pts);

    interp_A = nan(m,n);
    ind = 1;
    for i=1:m
        for j=1:n
            interp_A(i,j) = v(ind);
            ind = ind + 1;
        end
    end
    
    grid_obs = Grid(gmin, gmax, sz);
    grid_obs.SetData(interp_A);
end

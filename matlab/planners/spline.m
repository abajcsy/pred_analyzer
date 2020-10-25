%%   Computes a 3rd order spline of the form:
%      p(t) = a3(t/final_t)^3 + b3(t/final_t)^2 + c3(t/final_t) + d3
%      x(p) = a1p^3 + b1p^2 + c1p + d1
%      y(p) = a2p^2 + b2p^2 + c2p + d2
function curr_spline = spline(start, goal, final_t, num_waypts)
    % Get coefficients for the spline.
    [xc, yc, pc] = gen_coeffs(start, goal, final_t);
    
    % Unpack x coeffs.
    a1 = xc(1);
    b1 = xc(2);
    c1 = xc(3);
    d1 = xc(4);

    % Unpack y coeffs.
    a2 = yc(1);
    b2 = yc(2);
    c2 = yc(3);
    d2 = yc(4);

    % Unpack p coeffs.
    a3 = pc(1);
    b3 = pc(2);
    c3 = pc(3);
    d3 = pc(4);

    % Compute state trajectory at time steps using coefficients.
    xs = zeros([1,num_waypts]);
    ys = zeros([1,num_waypts]);
    ths = zeros([1,num_waypts]);
    xsdot = zeros([1,num_waypts]);
    ysdot = zeros([1,num_waypts]);
    ps = zeros([1,num_waypts]);
    psdot = zeros([1,num_waypts]);

    % Compute the control: u1 = linear vel, u2 = angular vel
    u1_lin_vel = zeros([1,num_waypts]);
    u2_ang_vel = zeros([1,num_waypts]);

    % Compute timestep between each waypt.
    dt = final_t/(num_waypts-1);
    idx = 1;
    t = 0;
    %while (t <= final_t)
    while idx <= num_waypts
        tnorm = t/final_t;
        
        if abs(t - 4*0.0716) < 0.001
            bla = 1;
        end

        % Compute (normalized) parameterized time var p and x,y and time derivatives of each.
        ps(idx)   = a3 * tnorm^3   + b3 * tnorm^2   + c3 * tnorm   + d3;  
        xs(idx)   = a1 * ps(idx)^3 + b1 * ps(idx)^2 + c1 * ps(idx) + d1;
        ys(idx)   = a2 * ps(idx)^3 + b2 * ps(idx)^2 + c2 * ps(idx) + d2;
        xsdot(idx)  = 3. * a1 * ps(idx)^2 + 2 * b1 * ps(idx) + c1;
        ysdot(idx)  = 3. * a2 * ps(idx)^2 + 2 * b2 * ps(idx) + c2;
        psdot(idx)  = 3. * a3 * tnorm^2 + 2 * b3 * tnorm + c3;
        ths(idx) = atan2(ysdot(idx), xsdot(idx));

        % Compute speed (wrt time variable p). 
        speed = sqrt(xsdot(idx)^2 + ysdot(idx)^2);

        xsddot = 6. * a1 * ps(idx) + 2. * b1 ;
        ysddot = 6. * a2 * ps(idx) + 2. * b2 ;

        % Linear Velocity (real-time): 
        %    u1(t) = u1(p(t)) * dp(tnorm)/dtorm * dtnorm/dt
        u1_lin_vel(idx) = speed * psdot(idx) / final_t;

        % Angular Velocity (real-time):
        %    u2(t) = u2(p(t)) * dp(tnorm)/dtorm * dtnorm/dt
        u2_ang_vel(idx) = (xsdot(idx)*ysddot - ysdot(idx)*xsddot)/speed^2 ...
                                * psdot(idx) / final_t;

        idx = idx + 1;
        t = t + dt;
    end
    
    curr_spline = {xs, ys, u1_lin_vel, u2_ang_vel, ths};
end

%% Generates spline coefficients. 
function [xc, yc, pc] = gen_coeffs(start, goal, final_t)
  % Extract states.
  x0 = start(1);
  y0 = start(2);
  th0 = start(3);
  v0 = start(4);

  xg = goal(1);
  yg = goal(2);
  thg = goal(3);
  vg = goal(4);

  % Set heuristic coefficients.
  f1 = v0 + sqrt((xg - x0)*(xg - x0) + (yg - y0)*(yg - y0));
  f2 = f1; 

  % Compute x(p(t)) traj coeffs.
  d1 = x0;
  c1 = f1*cos(th0);
  a1 = f2*cos(thg) - 2.*xg + c1 + 2.*d1;
  b1 = 3.*xg - f2*cos(thg) - 2.*c1 - 3.*d1;

  % Compute y(p(t))traj coeffs.
  d2 = y0;
  c2 = f1*sin(th0);
  a2 = f2*sin(thg) - 2.*yg + c2 + 2.*d2;
  b2 = 3.*yg - f2*sin(thg) - 2.*c2 - 3.*d2;

  % Compute p(t) coeffs.
  d3 = 0.0;
  c3 = (final_t * v0) / f1;
  a3 = (final_t * vg) / f2 + c3 - 2.;
  b3 = 1. - a3 - c3;

  xc = [a1, b1, c1, d1];
  yc = [a2, b2, c2, d2];
  pc = [a3, b3, c3, d3];
end
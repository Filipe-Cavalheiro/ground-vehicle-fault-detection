function coeffs = computeSplineCoefficients(t_points, values)
    % Number of points
    N = length(t_points);  
    h = diff(t_points); % h(i) = t_{i+1} - t_i for each interval

    num_unknowns = 4*(N-1); % 4 coefficients (a, b, c, d) per interval

    % Initialize matrix A (equations) and vector b (results)
    A = zeros(num_unknowns);  
    b = zeros(N, 1);  
    eq = 1; % equation counter

    % ===============================
    % Value constraints at each point
    % ===============================
    for i = 1:N-1
        % s_i(t_i) = value_i
        A(eq, (i-1)*4 + (1:4)) = [1, 0, 0, 0];
        b(eq) = values(i);
        eq = eq + 1;

        % s_i(t_{i+1}) = value_{i+1}
        dt = h(i); % time difference for current interval
        A(eq, (i-1)*4 + (1:4)) = [1, dt, dt^2, dt^3];
        b(eq) = values(i+1);
        eq = eq + 1;
    end

    % =================================================
    % First derivative continuity at internal points
    % (s_i'(t_{i+1}) = s_{i+1}'(t_{i+1}))
    % =================================================
    for i = 1:N-2
        dt = h(i);
        % Derivative of s_i at t_{i+1}
        A(eq, (i-1)*4 + (1:4)) = [0, 1, 2*dt, 3*dt^2];
        % Derivative of s_{i+1} at t_{i+1}
        A(eq, (i)*4 + (1:4)) = [0, -1, 0, 0];
        b(eq) = 0;
        eq = eq + 1;
    end

    % ==================================================
    % Second derivative continuity at internal points
    % (s_i''(t_{i+1}) = s_{i+1}''(t_{i+1}))
    % ==================================================
    for i = 1:N-2
        dt = h(i);
        % Second derivative of s_i at t_{i+1}
        A(eq, (i-1)*4 + (1:4)) = [0, 0, 2, 6*dt];
        % Second derivative of s_{i+1} at t_{i+1}
        A(eq, (i)*4 + (1:4)) = [0, 0, -2, 0];
        b(eq) = 0;
        eq = eq + 1;
    end

    % ===============================================
    % Natural spline boundary conditions:
    % Second derivative at endpoints = 0
    % ===============================================
    % s_0''(t0) = 0
    A(eq, 1:4) = [0, 0, 2, 0];
    b(eq) = 0;
    eq = eq + 1;

    % s_{N-2}''(tN) = 0
    dt = h(end);
    A(eq, end-3:end) = [0, 0, 2, 6*dt];
    b(eq) = 0;
    eq = eq + 1;

    % ===============================================
    % Solve the linear system for the coefficients
    % ===============================================
    coeffs = A\b;
end
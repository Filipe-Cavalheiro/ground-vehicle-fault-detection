function dxdt = ground_vehicle(~, x, linear_u, m, J, L)
% === Dynamics Function ===
    %linearization
    R = [cos(x(3)), sin(x(3));
        -sin(x(3)), cos(x(3))];

    aux1 = [m 0;0 J/L];
    aux2 = [-L*x(5)^2; x(4)*x(5)];
    u = aux1 * (R*linear_u-aux2);

    % State derivatives (nonlinear)
    dxdt = zeros(5,1);
    dxdt(1) = x(4) * cos(x(3));  % x_dot
    dxdt(2) = x(4) * sin(x(3));  % y_dot
    dxdt(3) = x(5);              % theta_dot
    dxdt(4) = u(1) / m;          % v_dot
    dxdt(5) = u(2) / J;          % w_dot
end
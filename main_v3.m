close all;
clearvars;
addpath(genpath('YALMIP-develop'))   % YALMIP, https://github.com/yalmip/YALMIP
addpath(genpath('ReachTool-main'))   % ReachTool (constrained zonotopes), provided in the course
addpath(genpath('Astar'))

%Variabels
rx = 2;
ry = 3;
center = [6,5];
vehicle_mass = 500;    % grams
vehicle_fuel = 500;    % grams
fuel_rate = 10;        % grams per step
J = 10;
L = 0.2;               % meters in front of center of mass
dt = 1;                % time step

cycles_with_fuel = vehicle_fuel / fuel_rate;
extra_steps = -1;     % Steps after fuel ends
total_steps = cycles_with_fuel + extra_steps;


%circle definition
th = 0:pi/50:2*pi;
x_obj = rx * cos(th) + center(1);
y_obj = ry * sin(th) + center(2);
% Vector from centroid to each point
dx = x_obj - center(1);
dy = y_obj - center(2);
dx_unit = dx ./ (rx/2);
dy_unit = dy ./ (ry/2);

% draw the trajectory of the vehicle
figure;
hold on;
axis equal; grid on;
fill(x_obj, y_obj, 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'b'); %object
drone = animatedline('Marker','o');
estimate = plot(1,1);
detected = false;

maximum_variance_x = 2; %Initial variance value
maximum_variance_y = 2;
real_xsim = zeros(1, total_steps);
real_ysim = zeros(1, total_steps);

%DEFINE THE 2-D MAP ARRAY
MAX_X=20;
MAX_Y=20;
%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y));

points = integer_points_in_circle(center, rx+maximum_variance_x, ry+maximum_variance_y);
points = min(max(points,1),MAX_X); %limit the points inside the MAP
MAP(points(:, 1),points(:,2)) = -1; %add new circle
h_points = plot(points(:, 1), points(:,2),'ro');

% Obtain Obstacle, Target and Robot Position
% Initialize the MAP with input values
% Obstacle=-1,Target = 0,Robot=1,Space=2
axis([1 MAX_X+1 1 MAX_Y+1])
grid on;
hold on;

% BEGIN Interactive Obstacle, Target, Start Location selection
xlabel('Please Select the Target using the Left Mouse button','Color','black');
but=0;
valid = 0;
while (but ~= 1 || valid ~= 1) %Repeat until the Left button is not clicked
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    if (~ismember(xval, points(:,1)) || ~ismember(yval, points(:,2)))
        valid = 1;
    end
end

xTarget=xval;   %X Coordinate of the Target
yTarget=yval;   %Y Coordinate of the Target

MAP(xval,yval) = 0;   %Initialize MAP with location of the target
plot(xval,yval,'gd');
text(xval,yval,'Target')

xlabel('Select Obstacles using the Left Mouse button,to select the last obstacle use the Right button','Color','blue');
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval) = -1;%Put on the closed list as well
    plot(xval,yval,'ro');
end

xlabel('Please Select the Vehicle initial position ','Color','black');
but=0;
valid = 0;
while (but ~= 1 || valid ~= 1) %Repeat until the Left button is not clicked
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    if (~ismember(xval, points(:,1)) || ~ismember(yval, points(:,2)))
        valid = 1;
    end
end

xStart=xval;%Starting Position
yStart=yval;%Starting Position
MAP(xval,yval)=1;
plot(xval,yval,'bo');
text(xval,yval,'Start')
%End of obstacle-Target pickup

% State vector: [x; y; ang_velocity; aceleration; ang_aceleration]
state = zeros(5, total_steps);
state(1:2,1) = [xStart;yStart];  % Initial position
y = zeros(3, total_steps-1);

A = [1 0 dt 0;
     0 1 0 dt;
     0 0 1 0;
     0 0 0 1];

B = [0.5*dt^2 0;
    0 0.5*dt^2;
    dt 0;
    0 dt]; 
C = [eye(3), zeros(3,2)];
C_linear = eye(2,4);
[m,n] = size(C_linear); % dimensions of output and state

% Define a feedback controller
K_pole = place(A,B,[0.2 0.3 0.5 0.4]);

xhat = zeros(4,total_steps + 1);
xhat(:,1) = [xStart;yStart;0;0];

Sigma = cell(1,total_steps + 2);
Sigma{1} = eye(4);

% covariance information for disturbances and noise
Q = 2 * eye(2);
R = 0.1 * eye(2);

% Observers - 1: Nominal model
X = cell(1, total_steps); 
X{1}.G = diag([xStart, yStart, 0, 0]);
X{1}.c = zeros(n,1);
X{1}.A = zeros(0,n);
X{1}.b = zeros(0,1);

% disturbance set
W.G = 1.25*eye(n);
W.c = zeros(n,1);
W.A = zeros(0,n);
W.b = zeros(0,1);

% noise set
V.G = 1.5*eye(2);
V.c = zeros(2,1);
V.A = zeros(0,2);
V.b = zeros(0,1);

error_total = 0;

% Loop through each time step
for k = 1:total_steps
    %update circle size
    MAP(points(:, 1),points(:,2)) = 1; %remove last circle
    delete(h_points);
    points = integer_points_in_circle(center, rx+maximum_variance_x, ry+maximum_variance_y);
    points = min(max(points,1),MAX_X); %limit the points inside the MAP
    MAP(points(:, 1),points(:,2)) = -1; %add new circle
    h_points = plot(points(:, 1), points(:,2),'ro');
    
    %get trajactory based on optimal path
    Optimal_path = [Astar(MAX_X, MAX_Y, MAP, xval, yval, xTarget, yTarget, xval, yval); [xval yval]];%take into account starting position
    x_trajectory = flip(Optimal_path(:,1)');
    y_trajectory = flip(Optimal_path(:,2)');
    
    % Parametric t values based on cumulative distance
    distances = sqrt(diff(x_trajectory).^2 + diff(y_trajectory).^2);
    t_points = [0, cumsum(distances)];
    
    % Compute spline coefficients for x(t) and y(t)
    coeffs_x = computeSplineCoefficients(t_points, x_trajectory);
    coeffs_y = computeSplineCoefficients(t_points, y_trajectory);
    
    % Evaluate splines at fine t grid
    tsim = linspace(t_points(1), t_points(end), total_steps);
    xsim = zeros(size(tsim));
    ysim = zeros(size(tsim));
    
    % Evaluate each interval
    for i = 1: length(t_points)-1
        idx = tsim >= t_points(i) & tsim <= t_points(i+1);
        dt = tsim(idx) - t_points(i);
    
        c_x = coeffs_x((i-1)*4 + (1:4));
        xsim(idx) = polyval([c_x(4) c_x(3) c_x(2) c_x(1)], dt);
    
        c_y = coeffs_y((i-1)*4 + (1:4));
        ysim(idx) = polyval([c_y(4) c_y(3) c_y(2) c_y(1)], dt);
    end

    real_xsim(k) = xsim(k);
    real_ysim(k) = ysim(k);
    ref = [xsim(k); ysim(k); 0; 0];
    
    if k == 1
        path_line = plot([real_xsim(1:k) xsim(k+1:end)], [real_ysim(1:k) ysim(k+1:end)], 'k-', 'LineWidth', 1.5);
    else
        set(path_line, 'XData', [real_xsim(1:k) xsim(k+1:end)], 'YData', [real_ysim(1:k) ysim(k+1:end)]);
    end 

    %control action 
    %error = [state(1:2,k);state(4,k)*cos(state(3,k));state(4,k)*sin(state(3,k))]-ref;
    error = [state(1:2,k);xhat(3:4,k)]-ref;
    error_total = error_total + error;
    linear_u = -K_pole*(error);

    % Kalman estimation
    % predict
    xhat(:,k+1) = A*xhat(:,k)+B*linear_u;
    Sigma{k+1} = A*Sigma{k}*A' + B*Q*B';

    total_mass = vehicle_mass + vehicle_fuel;
    % Integrate 1 step using the current state from state
    [~, x_next] = ode45(@(t, x) ground_vehicle(t, x, linear_u, total_mass, J, L), 0:1, state(:,k));

    % Store new state   
    state(:, k+1) = x_next(end, :);
    y(:,k+1) = C*state(:,k+1);
    
    % kalman filter update 
    ytilde = y(1:2,k+1)-C_linear*xhat(:,k+1);
    S = C_linear*Sigma{k+1}*C_linear'+R;
    K = Sigma{k+1}*C_linear'*inv(S);
    xhat(:,k+1) = xhat(:,k+1) + K*ytilde;
    Sigma{k+1} = (eye(4)-K*C_linear)*Sigma{k+1};
    
    % Run the observer
    X{k+1} = CZMinkowskiSum(CZLinMap(A,X{k},B*linear_u),W);
    Y = CZLinMap(-eye(2),V,y(1:2,k));
    X{k} = CZIntersect(C_linear,X{k},Y);

    [F,p] = compileCZ(X{k});

    setPoints = plot(F,p(1:2));

    if size(setPoints{1},2)==1 && ~detected
        fprintf("There is a fault at the iteration k = %d\n",k);
        detected = true;
    end
    
    xval = round(state(1,k+1));
    yval = round(state(2,k+1));
    xStart = xval;
    yStart = yval;

    if xval == xTarget && yval == yTarget
        disp("Target reached.");
        break;
    end

    % update the set-valued estimate
    set(estimate,'Xdata',setPoints{1}(1,:),'Ydata',setPoints{1}(2,:));
    maximum_variance_x = (max(setPoints{1}(1,:))-min(setPoints{1}(1,:)))/2;
    maximum_variance_y = (max(setPoints{1}(2,:))-min(setPoints{1}(2,:)))/2;

    addpoints(drone,state(1,k),state(2,k));
    drawnow;
end

%Obtained trajectory
figure;
hold on;
plot(nonzeros(y(1,2:end)), nonzeros(y(2,2:end)), 'b-', 'LineWidth', 1.5); %Obtained trajectory
xlabel('X Position'); ylabel('Y Position');
title('Vehicle Trajectory (True vs Estimated)');
axis equal; grid on;

disp("Error: " + sum(abs(error_total(1:2))));
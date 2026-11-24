close all;
clearvars;
addpath(genpath('YALMIP-develop'))   % YALMIP, https://github.com/yalmip/YALMIP
addpath(genpath('ReachTool-main'))   % ReachTool (constrained zonotopes), provided in the course
addpath(genpath('Astar'))

%variabels
r = 2;
center = [6,4];
vehicle_mass = 500;    % grams
vehicle_fuel = 500;    % grams
fuel_rate = 10;        % grams per step

cycles_with_fuel = vehicle_fuel / fuel_rate;
extra_steps = 0;     % Steps after fuel ends
total_steps = cycles_with_fuel + extra_steps;
dt = 1;

%circle definition
th = 0:pi/50:2*pi;
x_obj = r * cos(th) + center(1);
y_obj = r * sin(th) + center(2);
% Vector from centroid to each point
dx = x_obj - center(1);
dy = y_obj - center(2);
lengths = r/2;
dx_unit = dx ./ lengths;
dy_unit = dy ./ lengths;

% State-space matrices
A = [1 0 dt 0;
     0 1 0 dt;
     0 0 1 0;
     0 0 0 1];
B = [0.5*dt^2 0;
    0 0.5*dt^2;
    dt 0;
    0 dt]; 
C = kron([1,0],eye(2));

% draw the trajectory of the vehicle
figure;
hold on;
axis equal; grid on;
fill(x_obj, y_obj, 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'b'); %object
drone = animatedline('Marker','o');
estimate = plot(1,1);
detected = false;

maximum_variance = 0.1;
real_xsim = zeros(1, total_steps);
real_ysim = zeros(1, total_steps);

%DEFINE THE 2-D MAP ARRAY
MAX_X=20;
MAX_Y=20;
%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y));

points = integer_points_in_circle(center, r+maximum_variance, r+maximum_variance);
MAP(points(:, 1),points(:,2)) = -1; %add new circle
plot(points(:, 1)+0.5, points(:,2)+0.5,'ro');

% Obtain Obstacle, Target and Robot Position
% Initialize the MAP with input values
% Obstacle=-1,Target = 0,Robot=1,Space=2
j=0;
x_val = 1;
y_val = 1;
axis([1 MAX_X+1 1 MAX_Y+1])
grid on;
hold on;
n=0;%Number of Obstacles

% BEGIN Interactive Obstacle, Target, Start Location selection
xlabel('Please Select the Target using the Left Mouse button','Color','black');
but=0;
while (but ~= 1) %Repeat until the Left button is not clicked
    [xval,yval,but]=ginput(1);
end
xval=floor(xval);
yval=floor(yval);
xTarget=xval;%X Coordinate of the Target
yTarget=yval;%Y Coordinate of the Target

MAP(xval,yval)=0;%Initialize MAP with location of the target
plot(xval+.5,yval+.5,'gd');
text(xval+1,yval+.5,'Target')

xlabel('Select Obstacles using the Left Mouse button,to select the last obstacle use the Right button','Color','blue');
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=-1;%Put on the closed list as well
    plot(xval+.5,yval+.5,'ro');
end%End of While loop

xlabel('Please Select the Vehicle initial position ','Color','black');
but=0;
while (but ~= 1) %Repeat until the Left button is not clicked
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
end

xStart=xval;%Starting Position
yStart=yval;%Starting Position
MAP(xval,yval)=1;
plot(xval+.5,yval+.5,'bo');
text(xval+1,yval+.5,'Start')
%End of obstacle-Target pickup

% State vector: [x; y; vx; vy]
state = zeros(4, total_steps + 1);
state(1:2,1) = [xStart;yStart];  % Initial position
y = zeros(2, total_steps + 2);

% Define a feedback controller
K = place(A,B,[0.5 0.2 0.4 0.3]);

[m,n] = size(C); % dimensions of output and state

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

% Loop through each time step
for k = 1:total_steps
    MAP(points(:, 1),points(:,2)) = 1; %remove last circle
    
    points = integer_points_in_circle(center, r + maximum_variance, r+maximum_variance);
    points = max(points,1);
    MAP(points(:, 1),points(:,2)) = -1; %add new circle
    plot(points(:, 1)+0.5, points(:,2)+0.5,'ro');
    Optimal_path = Astar(MAX_X, MAX_Y, MAP,xval, yval, xTarget, yTarget, xStart, yStart);

    % j=size(Optimal_path,1);
    % %Plot the Optimal Path!
    % plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
    x_trajectory = flip(Optimal_path(:,1)');
    y_trajectory = flip(Optimal_path(:,2)');
    % Move points outward
    % x_trajectory = x_obj + dx_unit*maximum_variance;
    % y_trajectory = y_obj + dy_unit*maximum_variance;
    
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
    ref = [xsim; ysim; zeros(size(tsim)); zeros(size(tsim))];
    
    if k == 1
        path_line = plot([real_xsim(1:k) xsim(k+1:end)], [real_ysim(1:k) ysim(k+1:end)], 'k-', 'LineWidth', 1.5);
    else
        set(path_line, 'XData', [real_xsim(1:k) xsim(k+1:end)], 'YData', [real_ysim(1:k) ysim(k+1:end)]);
    end

    if k <= cycles_with_fuel
        u = -K*(state(:,k)-ref(:,k)); % controller
    else
        u = [0; 0];  % No acceleration after fuel runs out
    end
    state(:,k+1) = A * state(:,k) + B *u;
    y(:,k+1) = C*state(:,k+1);

    % Run the observer
    X{k+1} = CZMinkowskiSum(CZLinMap(A,X{k},B*u),W);
    Y = CZLinMap(-eye(2),V,y(:,k));
    X{k} = CZIntersect(C,X{k},Y);

    [F,p] = compileCZ(X{k});

    setPoints = plot(F,p(1:2));

    if size(setPoints{1},2)==1 && ~detected
        fprintf("There is a fault at the iteration k = %d\n",k);
        detected = true;
    end

    % update the set-valued estimate
    set(estimate,'Xdata',setPoints{1}(1,:),'Ydata',setPoints{1}(2,:));
    x_variance_length = (max(setPoints{1}(1,:))-min(setPoints{1}(1,:)))/2;
    y_variance_length = (max(setPoints{1}(2,:))-min(setPoints{1}(2,:)))/2;
    maximum_variance = sqrt(x_variance_length^2 + y_variance_length^2);

    addpoints(drone,state(1,k),state(2,k));
    %pause(1);
    drawnow;
end

%Object to go around, desired trajectory, obtained trajectory
figure;
hold on;
plot(state(1,:), state(2,:), 'b-', 'LineWidth', 1.5); %Obtained trajectory
xlabel('X Position'); ylabel('Y Position');
title('Vehicle Trajectory (True vs Estimated)');
legend('Object', 'Desired trajectory', 'True trajectory');
axis equal; grid on;
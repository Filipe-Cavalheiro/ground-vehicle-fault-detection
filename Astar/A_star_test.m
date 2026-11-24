%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A* ALGORITHM Demo
% Interactive A* search demo
% 04-26-2005
%   Copyright 2009-2010 The MathWorks, Inc.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;

%DEFINE THE 2-D MAP ARRAY
MAX_X=20;
MAX_Y=20;
%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y));

c = [6, 6];  % Circle centered at origin
r = 2.2;
points = integer_points_in_circle(c, r);
MAP(points(:, 1),points(:,2)) = -1;
plot(points(:, 1)+0.5, points(:,2)+0.5,'ro');
viscircles(c, r, 'Color', 'r');

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

Optimal_path = Astar(MAX_X, MAX_Y, MAP,xval, yval, xTarget, yTarget, xStart, yStart);

j=size(Optimal_path,1);
%Plot the Optimal Path!
plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);

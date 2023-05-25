%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% fn_feedback_control.m
% 
% Purpose: Function to control vehicle motion relative to intended target path
% 
% Input [x0,dt,path]: initial pose (x0)
%                     sampling time (dt)
%                     path
%
% Output [x,y,z,roll,pitch,yaw]: pose at current timestep
%
% Primary Developer Contact Information:
% Heath Cottrill
% Student
% Statler College of Engineering & Mineral Resources
% Dept. Mechanical and Aerospace Engineering
% West Virginia University (WVU)
% hsc0009@mix.wvu.edu
%
% Development History
% Date              Developer        Comments
% ---------------   -------------    --------------------------------
% May 20, 2023      H. Cottrill      Initial implemention
%

function [x, y, z, roll, pitch, yaw] = fn_feedback_control(x0, dt, path)

% Gains for proportional control
kr = 1;
kp = 1;
ky = 0.1;

% Set limits on turn severity
max_roll_steering_angle_limit = pi/6;
min_roll_steering_angle_limit = -pi/6;
max_pitch_steering_angle_limit = pi/6;
min_pitch_steering_angle_limit = -pi/6;
max_yaw_steering_angle_limit = pi/6;
min_yaw_steering_angle_limit = -pi/6;

% Establish the dimensions of the UAV for turn radius
l_roll = 10;
l_pitch = 10;
l_yaw = 10;

% Extract the current states
x_current = x0(1);
y_current = x0(2);
z_current = x0(3);
Euler_current = x0(4:6);
v = 30;

% Determine closest point of path that is forward
difference = sqrt((path(:,1)-x_current).^2 + (path(:,2)-y_current).^2 + (path(:,3)-z_current).^2);
[~,min_index] = min(difference);

if min_index >= length(difference)-1
    goal = [path(length(difference),1)-x_current path(length(difference),2)-y_current path(length(difference),3)-z_current];
else
    goal = [path(min_index+1,1)-x_current path(min_index+1,2)-y_current path(min_index+1,3)-z_current];
end

if isnan(goal)
    disp('hello')
elseif norm(goal) == 0
    disp('hello')
end

% Calculate the difference between current and goal states
r = vrrotvec([1 0 0],goal);
m = vrrotvec2mat(r);
theta = DCM_to_Euler_Cottrill(m,321);

yaw_goal = theta(1);
pitch_goal = theta(2);
roll_goal = theta(3);

roll_difference = angdiff(roll_goal,Euler_current(1));
pitch_difference = angdiff(pitch_goal,Euler_current(2));
yaw_difference = angdiff(yaw_goal,Euler_current(3));

% Apply portional control
roll_steering_angle = -kr*roll_difference;
pitch_steering_angle = -kp*pitch_difference;
yaw_steering_angle = -ky*yaw_difference;

% Ensure results do not exceed capability
if  roll_steering_angle > max_roll_steering_angle_limit
    roll_steering_angle = max_roll_steering_angle_limit;
elseif roll_steering_angle < min_roll_steering_angle_limit
    roll_steering_angle  = min_roll_steering_angle_limit;
end

if  pitch_steering_angle > max_pitch_steering_angle_limit
    pitch_steering_angle = max_pitch_steering_angle_limit;
elseif pitch_steering_angle < min_pitch_steering_angle_limit
    pitch_steering_angle  = min_pitch_steering_angle_limit;
end

if  yaw_steering_angle > max_yaw_steering_angle_limit
    yaw_steering_angle = max_yaw_steering_angle_limit;
elseif yaw_steering_angle < min_yaw_steering_angle_limit
    yaw_steering_angle  = min_yaw_steering_angle_limit;
end

% Non-dimensionalize command
unit_roll_rate = tan(roll_steering_angle) / l_roll;
unit_pitch_rate = tan(pitch_steering_angle) / l_pitch;
unit_yaw_rate = tan(yaw_steering_angle) / l_yaw;

% Calculate update angles
roll_rate = v*unit_roll_rate;
droll = roll_rate*dt;
roll = Euler_current(1) + droll;
pitch_rate = v*unit_pitch_rate;
dpitch = pitch_rate*dt;
pitch = Euler_current(2) + dpitch;
yaw_rate = v*unit_yaw_rate;
dyaw = yaw_rate*dt;
yaw = Euler_current(3) + dyaw;

% Determine new positions
vx = v*cos(pitch)*cos(yaw);
vy = v*(sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw));
vz = v*(cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw));
dx = vx*dt;
dy = vy*dt;
dz = vz*dt;
x = x_current+dx;
y = y_current+dy;
z = z_current+dz;

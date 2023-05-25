%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% fn_propagate.m
% 
% Purpose: Function to propogate pose based on previous pose and time step
% 
% Input [x,y,z,roll,pitch,yaw,vel,dyaw,dpitch,droll,dt]: pose at timestep k
%       x: x position
%       y: y position
%       z: z position
%       roll: rotation around the front-to-back axis
%       pitch: rotation around the side-to-side axis
%       yaw: rotation around the vertical axis 
%		vel: velocity
%		droll: roll rate
%		dpitch: pitch rate
%		dyaw: yaw rate
%		dt: sampling time
%
% Output [x,y,z,roll,pitch,yaw]: pose at timestep k+1
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

%%

function [x,y,z,roll,pitch,yaw] = fn_propagate(x,y,z,roll,pitch,yaw,vel,droll,dpitch,dyaw,dt)
	roll = roll + dt*droll;
    pitch = pitch + dt*dpitch;
    yaw = yaw + dt*dyaw;
    x = x + dt*vel*(cos(pitch)*cos(yaw));
	y = y + dt*vel*(sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw));
    z = z + dt*vel*(cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw));
end

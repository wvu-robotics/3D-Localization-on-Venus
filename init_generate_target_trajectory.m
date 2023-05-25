%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% init_generate_target_trajectory.m
% 
% Purpose: Initialize Aerobot's Position, Orientation, and Velocity
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

%% Initial Offset

offset_x = simulation.initoffset*cos(2*pi*rand)*cos(2*pi*rand); % offset x
offset_y = simulation.initoffset*sin(2*pi*rand)*cos(2*pi*rand); % offset y
offset_z = simulation.initoffset*cos(2*pi*rand); % offset z

%% Noise

agent.sigma_Velocity = simulation.sigma_Velocity;
agent.sigma_Roll_Rate = simulation.sigma_Roll_Rate;
agent.sigma_Pitch_Rate = simulation.sigma_Pitch_Rate;
agent.sigma_Yaw_Rate = simulation.sigma_Yaw_Rate;
agent.bias_Velocity = normrnd(0,simulation.bias_Velocity);
agent.bias_Roll_Rate = normrnd(0,simulation.bias_Roll_Rate);
agent.bias_Pitch_Rate = normrnd(0,simulation.bias_Pitch_Rate);
agent.bias_Yaw_Rate = normrnd(0,simulation.bias_Yaw_Rate);

%% Truth

% Velocity
agent.tvel(1) = 30; 

% Starting position
agent.tpx(1) = path(1,1); % init x truth
agent.tpy(1) = path(1,2); % init y truth
agent.tpz(1) = path(1,3); % init z truth

% Starting Orientation
r = vrrotvec([1 0 0],[path(3,1)-path(2,1) path(3,2)-path(2,2) path(3,3)-path(2,3)]);
m = vrrotvec2mat(r);
theta = DCM_to_Euler_Cottrill(m,321);

agent.tyaw(1) = theta(1); % init yaw truth
agent.tpitch(1) = theta(2); % init pitch truth
agent.troll(1) = theta(3); % init roll truth
agent.tdroll(1) = 0; % init roll rate truth
agent.tdpitch(1) = 0; % init pitch rate truth
agent.tdyaw(1) = 0; % init yaw rate truth
agent.tvx(1) = agent.tvel(1)*cos(agent.tpitch(1))*cos(agent.tyaw(1)); % init velocity in x axis truth
agent.tvy(1) = agent.tvel(1)*(sin(agent.troll(1))*sin(agent.tpitch(1))*cos(agent.tyaw(1))-cos(agent.troll(1))*sin(agent.tyaw(1))); % init velocity in y axis truth
agent.tvz(1) = agent.tvel(1)*(cos(agent.troll(1))*sin(agent.tpitch(1))*cos(agent.tyaw(1))+sin(agent.troll(1))*sin(agent.tyaw(1))); % init velocity in z axis truth

%% Estimates

agent.vel(1) = agent.tvel(1);
agent.vx(1) = agent.tvx(1);
agent.vy(1) = agent.tvy(1);
agent.vz(1) = agent.tvz(1);
agent.px(1) = agent.tpx(1)+normrnd(0,simulation.initUncertainty)+offset_x;
agent.py(1) = agent.tpy(1)+normrnd(0,simulation.initUncertainty)+offset_y;
agent.pz(1) = agent.tpz(1)+normrnd(0,simulation.initUncertainty)+offset_z;
agent.roll(1) = agent.troll(1);
agent.pitch(1) = agent.tpitch(1);
agent.yaw(1) = agent.tyaw(1);
agent.droll(1) = agent.tdroll(1);
agent.dpitch(1) = agent.tdpitch(1);
agent.dyaw(1) = agent.tdyaw(1);

%% Clean

clear offset_x offset_y offset_z r m theta
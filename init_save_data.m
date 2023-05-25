%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% init_save_data.m
% 
% Purpose: Initialize Data Save Procedure
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
%% Save Data

mn.px(1) = agent.px(1);
mn.py(1) = agent.py(1);
mn.pz(1) = agent.pz(1);
mn.pitch(1) = agent.pitch(1);
mn.roll(1) = agent.roll(1);
mn.yaw(1) = agent.yaw(1);

pr.px(1) = agent.px(1);
pr.py(1) = agent.py(1);
pr.pz(1) = agent.pz(1);
pr.pitch(1) = agent.pitch(1);
pr.roll(1) = agent.roll(1);
pr.yaw(1) = agent.yaw(1);

%% Clean

clear i
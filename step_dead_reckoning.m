%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% step_dead_reckoning.m
% 
% Purpose: Update pose based on propagation function
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

%% Propagate Pose

[x,y,z,roll,pitch,yaw] = fn_propagate(agent.px(simulation.i-1),...
    agent.py(simulation.i-1),...
    agent.pz(simulation.i-1),...
    agent.roll(simulation.i-1),...
    agent.pitch(simulation.i-1),...
    agent.yaw(simulation.i-1),...
    agent.vel(simulation.i),...
    agent.droll(simulation.i),...
    agent.dpitch(simulation.i),...
    agent.dyaw(simulation.i),...
    simulation.Ts);

agent.px(simulation.i)=x;
agent.py(simulation.i)=y;
agent.pz(simulation.i)=z;
agent.roll(simulation.i)=roll;
agent.pitch(simulation.i)=pitch;
agent.yaw(simulation.i)=yaw;

clear x y z roll pitch yaw
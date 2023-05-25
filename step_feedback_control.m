%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% step_feedback_control.m
% 
% Purpose: Determine true and noisy pose from feedback control function
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

%% Previous Orientation for Step Feedback Control
if simulation.i <= 1
    fc.x = px(simulation.i - 1);
    fc.y = py(simulation.i - 1);
    fc.z = pz(simulation.i - 1);
    fc.roll = roll(simulation.i - 1);
    fc.pitch = pitch(simulation.i - 1);
    fc.yaw = yaw(simulation.i - 1);
else
    fc.x = mn.px(simulation.i - 1); 
    fc.y = mn.py(simulation.i - 1);
    fc.z = pr.pz(simulation.i - 1);
    fc.roll = agent.roll(simulation.i - 1);
    fc.pitch = agent.pitch(simulation.i - 1);
    fc.yaw = agent.yaw(simulation.i - 1);
end

fc.x0 = [fc.x fc.y fc.z fc.roll fc.pitch fc.yaw];
    
%% Call Correspondng Function
% function [x y z roll pitch yaw] = fn_feedback_control(x0, dt, path)
[fc.posex, fc.posey, fc.posez, fc.poseroll, fc.posepitch, fc.poseyaw] = fn_feedback_control(fc.x0,simulation.dt,path);

% True orientation from feedback control
agent.troll(simulation.i) = agent.troll(simulation.i-1) + (fc.poseroll-fc.x0(4));	% truth roll from feedback control (rad)
agent.tpitch(simulation.i) = agent.tpitch(simulation.i-1) + (fc.posepitch-fc.x0(5));	% truth pitch from feedback control (rad)
agent.tyaw(simulation.i) = agent.tyaw(simulation.i-1) + (fc.poseyaw-fc.x0(6));	% truth yaw from feedback control (rad)
agent.tpx(simulation.i) = agent.tpx(simulation.i-1) + sqrt((fc.posex-fc.x0(1))^2 + (fc.posey-fc.x0(2))^2 + (fc.posez-fc.x0(3))^2) * cos(agent.tyaw(simulation.i)) * cos(agent.tpitch(simulation.i)); %truth x from feedback control
agent.tpy(simulation.i) = agent.tpy(simulation.i-1) + sqrt((fc.posex-fc.x0(1))^2 + (fc.posey-fc.x0(2))^2 + (fc.posez-fc.x0(3))^2) * (sin(agent.troll(simulation.i)) * sin(agent.tpitch(simulation.i)) * cos(agent.tyaw(simulation.i)) - cos(agent.troll(simulation.i)) * sin(agent.tyaw(simulation.i))); %truth y from feedback control
agent.tpz(simulation.i) = agent.tpz(simulation.i-1) + sqrt((fc.posex-fc.x0(1))^2 + (fc.posey-fc.x0(2))^2 + (fc.posez-fc.x0(3))^2) * (cos(agent.troll(simulation.i)) * sin(agent.tpitch(simulation.i)) * cos(agent.tyaw(simulation.i)) + sin(agent.troll(simulation.i)) * sin(agent.tyaw(simulation.i))); %truth z from feedback control

% Truth rates of change from feedback control
agent.tvel(simulation.i) = sqrt((fc.posex-fc.x0(1))^2 + (fc.posey-fc.x0(2))^2 + (fc.posez-fc.x0(3))^2) / simulation.Ts;
agent.tdroll(simulation.i) = (fc.poseroll-fc.x0(4)) / simulation.Ts;
agent.tdpitch(simulation.i) = (fc.posepitch-fc.x0(5)) / simulation.Ts;
agent.tdyaw(simulation.i) = (fc.poseyaw-fc.x0(6)) / simulation.Ts;

%% Measurements (noisy)

% Odometry
agent.vel(simulation.i) = normrnd(agent.tvel(simulation.i),simulation.sigma_Velocity) + agent.bias_Velocity;
agent.dpitch(simulation.i) = normrnd(agent.tdpitch(simulation.i),simulation.sigma_Pitch_Rate) + agent.bias_Pitch_Rate;
agent.droll(simulation.i) = normrnd(agent.tdroll(simulation.i),simulation.sigma_Roll_Rate) + agent.bias_Roll_Rate;
agent.dyaw(simulation.i) = normrnd(agent.tdyaw(simulation.i),simulation.sigma_Yaw_Rate) + agent.bias_Yaw_Rate;

%% Clean

clear fc
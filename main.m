%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% main.m
% 
% Purpose: Single script to change parameters and run simulation            
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
%% Clean Workspace

% Clear and close previous work
clear 
close
clc
        
% Save seed for testing later
s = rng;

%% Simulation Parameters

% Path
load('test_path_diverse.mat','path')
% load('test_path_flat.mat','path')

% Time
simulation.Time=60*60; % flight duration (second)
simulation.accumulatedTime = 0; % initial time elapsed (second)
simulation.Ts=0.1; % dead reckoning update rate (second)
simulation.update_Ts_1=0.2; % ranging measurement update rate (second)
simulation.update_Ts_2=0.2; % pressure measurement update rate (second)

% Standard Deviation
simulation.sigma_Velocity=1; % standard deviation of velocity error (m/s)
simulation.sigma_Roll_Rate=0.05*pi/180; % standard deviation of roll rate error (rad/s)
simulation.sigma_Pitch_Rate=0.05*pi/180; % standard deviation of pitch rate error (rad/s)
simulation.sigma_Yaw_Rate=0.05*pi/180; % standard deviation of yaw rate error (rad/s)
simulation.sigma_Range = 10; % standard deviation of ranging measurement error (m)
simulation.sigma_Pressure= 10/1000; % standard deviation of pressure measurement error (bar) = 1,000 Pa

% Bias
simulation.bias_Velocity = 0.1; % bias of velocity turn on bias (m/s)
simulation.bias_Roll_Rate=0.005*simulation.Time/3600*pi/180; % bias of roll rate turn on bias (rad/s)
simulation.bias_Yaw_Rate=0.005*simulation.Time/3600*pi/180; % bias of yaw rate turn on bias (rad/s)
simulation.bias_Pitch_Rate=0.005*simulation.Time/3600*pi/180; % bias of pitch rate turn on bias (rad/s)
simulation.initoffset=0;
simulation.initUncertainty=0;

% Particle Filter
simulation.Map = 1;    % altitude of 4641 m
simulation.npf = 10000; % number of particles
simulation.threshold_resample = 0.5*simulation.npf;	%if the number of effective particles below threshold, do resample

% Feedback Control
simulation.dt = simulation.Ts;	%sample time

% Display Simulation Parameters
disp(simulation);
tic

%% Initialization

run('init_generate_target_trajectory.m');
run('init_pf_1.m');
run('init_pf_2.m');
run('init_save_data.m');

%% Simulation

disp('Performing simulation...');
simulation.i=2;

while simulation.accumulatedTime < simulation.Time

    % Execute Step Scripts
    run('step_feedback_control.m');
    run('step_dead_reckoning.m');
    run('step_pf_1.m');
    run('step_pf_2.m');
    run('step_save_data.m');

    % Update Simulation Variables
    simulation.i = simulation.i + 1;
    simulation.accumulatedTime = simulation.accumulatedTime + simulation.Ts;

    % Show Progress
    if(mod(simulation.i,250)==0)
        progress = simulation.accumulatedTime/(simulation.Time)*100;
        disp(progress);
    end
end

%% Plots

disp('Plotting Figures...')
run('plot_trajectories');
run('plot_errors');

clear progress
disp('Simulation Complete!')

toc

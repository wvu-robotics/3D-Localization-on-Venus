%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% init_pf_2.m
% 
% Purpose: Initialize Barometry-Based Particle Filter
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

%% Display

disp('Initializing PF_2...');

%% Build Model

% Particle Filter Inputs
pf.nx_2 = 6; % state dimension (x,y,z,roll,pitch,yaw)
pf.ny_2 = 1; % measurement dimension
pf.x0_2 = [agent.px(1); agent.py(1); agent.pz(1); agent.roll(1); agent.pitch(1); agent.yaw(1)]; % initial pose with uncertainty
pf.npf_2 = simulation.npf; % number of particles in the particle filter

% Particle Filter States
pf.P0_2 = zeros(pf.nx_2); 
pf.P0_2(1,1) = 0;
pf.P0_2(2,2) = 0;
pf.P0_2(3,3) = 1;
pf.P0_2(4,4) = 0;
pf.P0_2(5,5) = 0;
pf.P0_2(6,6) = 0;

% Particle Filter Outputs
pf.x_2 = repmat(pf.x0_2,1,pf.npf_2) + pf.P0_2 * randn(pf.nx_2,pf.npf_2);
pf.w_2 = ones(pf.npf_2,1)/pf.npf_2; % weight of particle
pf.q_2 = ones(pf.npf_2,1)/pf.npf_2; % weight of particle at each time step
pf.xf_2(:,1) = pf.x0_2; % output of particle filter
pf.divergence_threshold_2 = 0.001;

%% Propagate Matrix Update Based on Noise

if(simulation.sigma_Roll_Rate == 0.5*pi/180 || simulation.sigma_Roll_Rate > 0.5*pi/180)
    simulation.sigma_Roll_Rate_temp = simulation.sigma_Roll_Rate;
else
    simulation.sigma_Roll_Rate_temp = simulation.sigma_Roll_Rate *10;
end

if(simulation.sigma_Pitch_Rate == 0.5*pi/180 || simulation.sigma_Pitch_Rate > 0.5*pi/180)
    simulation.sigma_Pitch_Rate_temp = simulation.sigma_Pitch_Rate;
else
    simulation.sigma_Pitch_Rate_temp = simulation.sigma_Pitch_Rate *10;
end

if(simulation.sigma_Yaw_Rate == 0.5*pi/180 || simulation.sigma_Yaw_Rate > 0.5*pi/180)
    simulation.sigma_Yaw_Rate_temp = simulation.sigma_Yaw_Rate;
else
    simulation.sigma_Yaw_Rate_temp = simulation.sigma_Yaw_Rate *10;
end

simulation.sigma_Velocity_temp = simulation.sigma_Velocity * 10;

clear i

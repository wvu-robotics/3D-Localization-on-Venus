%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% init_pf_1.m
% 
% Purpose: Initialize Terrain-aided Navigation Particle Filter
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

disp('Initializing PF_1...');

%% Load Map Data

load 'Venus_map.mat'
f = griddedInterpolant(Venus_xm',Venus_ym',Venus_map');

%% Symbolic Representation

m_formulas.var(1) = sym(sprintf('x'));
m_formulas.var(2) = sym(sprintf('y'));
m_formulas.var(3) = sym(sprintf('z'));
m_formulas.Ts = sym(sprintf('Ts'));

m_formulas.pf(1) = sym(sprintf('pfx')); % x component of position
m_formulas.pf(2) = sym(sprintf('pfy')); % y component of position
m_formulas.pf(3) = sym(sprintf('pfz')); % z component of position
m_formulas.pf(4) = sym(sprintf('pfroll')); % roll angle
m_formulas.pf(5) = sym(sprintf('pfpitch'));	% pitch angle
m_formulas.pf(6) = sym(sprintf('pfyaw')); % yaw angle

m_formulas.vel = sym(sprintf('vel')); % velocity from visual inertial odom
m_formulas.droll = sym(sprintf('droll')); % roll rate from visual inertial odom
m_formulas.dpitch = sym(sprintf('dpitch')); % pitch rate from visual inertial odom
m_formulas.dyaw = sym(sprintf('dyaw')); % yaw rate from visual inertial odom

%% Dynamic Function

m_formulas.f(4,:) = m_formulas.pf(4) + m_formulas.Ts * m_formulas.droll; 
m_formulas.f(5,:) = m_formulas.pf(5) + m_formulas.Ts * m_formulas.dpitch; 
m_formulas.f(6,:) = m_formulas.pf(6) + m_formulas.Ts * m_formulas.dyaw; 
m_formulas.f(1,:) = m_formulas.pf(1) + m_formulas.Ts * m_formulas.vel .* (cos(m_formulas.f(5,:)) .* cos(m_formulas.f(6,:)));
m_formulas.f(2,:) = m_formulas.pf(2) + m_formulas.Ts * m_formulas.vel .* (sin(m_formulas.f(4,:)) .* sin(m_formulas.f(5,:)).* cos(m_formulas.f(6,:)) - cos(m_formulas.f(4,:)) .* sin(m_formulas.f(6,:)));
m_formulas.f(3,:) = m_formulas.pf(3) + m_formulas.Ts * m_formulas.vel .* (cos(m_formulas.f(4,:)) .* sin(m_formulas.f(5,:)).* cos(m_formulas.f(6,:)) + sin(m_formulas.f(4,:)) .* sin(m_formulas.f(6,:)));

m_formulas.fnf = matlabFunction(m_formulas.f);

%% Build Model

% Particle Filter Inputs
pf.nx_1 = 6; % state dimension (x,y,z,roll,pitch,yaw)
pf.ny_1 = 1; % measurement dimension
pf.x0_1 = [agent.px(1); agent.py(1); agent.pz(1); agent.roll(1); agent.pitch(1); agent.yaw(1)]; % initial pose with uncertainty
pf.npf_1 = simulation.npf; % number of particles in the particle filter

% Particle Filter States
pf.P0_1 = zeros(pf.nx_1); 
pf.P0_1(1,1) = 1;
pf.P0_1(2,2) = 1;
pf.P0_1(3,3) = 0;
pf.P0_1(4,4) = 0;
pf.P0_1(5,5) = 0;
pf.P0_1(6,6) = 0;

% Particle Filter Outputs
pf.x_1 = repmat(pf.x0_1,1,pf.npf_1) + pf.P0_1 * randn(pf.nx_1,pf.npf_1);
pf.w_1 = ones(pf.npf_1,1)/pf.npf_1; % weight of particle
pf.q_1 = ones(pf.npf_1,1)/pf.npf_1; % weight of particle at each time step
pf.xf_1(:,1) = pf.x0_1; % output of particle filter
pf.divergence_threshold_1 = 0.001;

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

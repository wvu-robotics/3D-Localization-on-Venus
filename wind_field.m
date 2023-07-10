function [e,n,u] = wind_field(z)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% wind_field.m
% 
% Purpose: Determine wind field based in ENU coordinate framed based on altitude
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
% July 10, 2023     H. Cottrill      Initial implemention

% Extract altitude
R = 6051800.0000;
z = z-R;

% Seed for randomization
rng(round(z));

% Eastward Zonal Wind

% Confine to available range
if z > 75000
    z = 75000;
elseif z < 45000
    z = 45000;
end

% Altitude in meters
altitudes = transpose([45000,50000,55000,60000,65000,70000,75000]);
    
% Absolute wind speed as function of altitude in m/s
absolute_wind_speed = transpose([51,61,60,80,94,92,84]);

% Variable wind speed as function of altitude in m/s
variable_wind_speed = transpose([15,25,25,30,30,30,25]);

% Linearly interpolate wind speeds
e_mu = interp1(altitudes,absolute_wind_speed,z,'linear');
e_sigma = interp1(altitudes,variable_wind_speed,z,'linear');
e = normrnd(e_mu,e_sigma);

% Northern Meridional
n_mu = 0;
n_sigma = 2.5;
n = normrnd(n_mu,n_sigma);

% Vertical Winds
u_mu = 0;
u_sigma = 0.01;
u = normrnd(u_mu,u_sigma);

end
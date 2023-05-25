%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% air_pressure.m
% 
% Purpose: Determine air pressure based on altitude
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
% May 20, 2023   H. Cottrill      Initial implemention
 
function [pressure] = air_pressure(z)

% Altitude in meters
altitudes=transpose([0,5000,10000,15000,20000,25000,30000,35000,40000,45000,50000,55000,60000,65000,70000,80000,90000,100000]);
    
% Pressure in bars (1 bar = 100,000 Pa)
pressures=transpose([92.1,66.65,47.39,33.04,22.52,14.93,9.851,5.917,3.501,1.979,1.066,0.5314,0.2357,0.0977,0.0369,0.0048,3.736E-04,2.66E-05]);
    
% Linearly interpolate pressure values
pressure=interp1(altitudes,pressures,z,'linear');

% If the altitude exceeds the model, set the pressure value as the minimum
if z > 100000
    pressure = 2.66E-05;
end

end
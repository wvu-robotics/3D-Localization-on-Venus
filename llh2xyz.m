function xyz = llh2xyz(llh)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% llh2xyz.m
% 
% Purpose: Convert from latitude, longitude and height to ECEF cartesian coordinates
%
% Input: llh(1) = latitude in radians
%        llh(2) = longitude in radians
%        llh(3) = height above ellipsoid in meters
%
% Output: xyz(1) = ECEF x-coordinate in meters
%         xyz(2) = ECEF y-coordinate in meters
%         xyz(3) = ECEF z-coordinate in meters
%
% Source: J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates to geodetic coordinates," 
%         in IEEE Transactions on Aerospace and Electronic Systems, vol. 30, no. 3, pp. 957-961, 
%         July 1994, doi: 10.1109/7.303772.
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
%

% Geodetic coordinates
phi = llh(1);
lambda = llh(2);
h = llh(3);

% Venus equatorial radius in meters
a = 6051800.0000;

% Venus polar radius in meters
b = 6051800.0000;

% Eccentricity of Venus surface
e = sqrt(1-b^2/a^2);

% Prime vertical radius of curvature
R = a/sqrt(1-e^2*sin(phi)^2);

% Geocentric Cartesian coordinates 
x = (R+h)*cos(phi)*cos(lambda);
y = (R+h)*cos(phi)*sin(lambda);
z = (R+h-e^2*R)*sin(phi);

% Concatenate
xyz=[x;y;z];

end
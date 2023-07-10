function llh = xyz2llh(xyz)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% xyz2llh.m
% 
% Purpose: Convert from ECEF cartesian coordinates to latitude, longitude and height
% 
% Input: xyz(1) = ECEF x-coordinate in meters
%        xyz(2) = ECEF y-coordinate in meters
%        xyz(3) = ECEF z-coordinate in meters
%
% Output: llh(1) = latitude in radians
%         llh(2) = longitude in radians
%         llh(3) = height in meters
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

% Geocentric Cartesian coordinates 
x = xyz(1);
y = xyz(2);
z = xyz(3);

% Venus equatorial radius in meters
a = 6051800.0000;

% Venus polar radius in meters
b = 6051800.0000;

% Eccentricity of Venus surface
e = sqrt(1-b^2/a^2);
ep2 = (a^2-b^2)/b^2;

% Equation proposed by Heikkinen, M. in 1982
r = sqrt(x^2+y^2);
F = 54*b^2*z^2;
G = r^2+(1-e^2)*z^2-e^2*(a^2-b^2);
c = (e^4*F*r^2)/G^3;
s = (1+c+sqrt(c^2+2*c))^(1/3);
P = F/(3*(s+1/s+1)^2*G^2);
Q = sqrt(1+2*e^4*P);
ro = -(P*e^2*r)/(1+Q)+sqrt(a^2/2*(1+1/Q)-(P*(1-e^2)*z^2)/(Q*(1+Q))-P*r^2/2);
U = sqrt((r-e^2*ro)^2+z^2);
V = sqrt((r-e^2*ro)^2+(1-e^2)*z^2);
zo = (b^2*z)/(a*V);

% Geodetic coordinates
phi = atan((z+ep2*zo)/r);
lambda = 2*atan2(sqrt(x^2+y^2)-x,y);
h = U*(1-b^2/(a*V));

% Concatenate
llh = [phi; lambda; h];

end

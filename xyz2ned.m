function ned = xyz2ned(llh,xyz,orgxyz)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% xyz2ned.m
% 
% Purpose: Convert from cartesian coordinates to rectangular local-level-tangent 
%          ('North'-'East'-'Down') coordinates.
%
% Input: llh(1) = latitude in radians
%        llh(2) = longitude in radians
%        llh(3) = height in meters
%
%        xyz(1) = ECEF x-coordinate in meters
%        xyz(2) = ECEF y-coordinate in meters
%        xyz(3) = ECEF z-coordinate in meters
%
%        orgxyz(1) = ECEF x-coordinate of local origin in meters
%	     orgxyz(2) = ECEF y-coordinate of local origin in meters
%	     orgxyz(3) = ECEF z-coordinate of local origin in meters
%
% Output: ned(1) = 'North' coordinate relative to local origin (meters)
%		  ned(2) = 'East' coordinate relative to local origin (meters)
%		  ned(3) = 'Down' coordinate relative to local origin (meters)
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

% Return error if not enough inputs
if nargin<2
    error('insufficient number of input arguments')
end

% Geodetic coordinates
phi = llh(1);
lambda = llh(2);

% Rotation Matrix
R = [-sin(lambda)           cos(lambda)          0;
     -cos(lambda)*sin(phi) -sin(lambda)*sin(phi) cos(phi);
      cos(lambda)*cos(phi)  sin(lambda)*cos(phi) sin(phi)];

% Cartesian from origin
enu = R*(xyz-orgxyz);

% Conversion from ENU to NED
enu2ned = [0 1 0;
           1 0 0;
           0 0 -1];
ned = enu2ned*enu;

end
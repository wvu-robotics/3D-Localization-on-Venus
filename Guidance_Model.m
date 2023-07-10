%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% Guidance_Model.m
% 
% Purpose: Determine angular velocity and acceleration based on wind field
% 
% Input [ins_att,ins_vel,ins_pos,ins_pos_xyz,dt]:
%       ins_att: Euler angles
%       ins_vel: Linear velocities
%       ins_pos: Latitude, Longitude, Height 
%       ins_pos_xyz: x, y, z
%		dt: Sampling time
%
% Output [omega_b_ib,v_ib_b]:
%        omega_b_ib: Angular rate (gyroscope data)
%        v_ib_b: Acceleration (accelerometer data)
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
% July 10, 2023      H. Cottrill      Initial implemention
%

function [omega_b_ib,f_ib_b,acc_g,acc_r,acc_w] = Guidance_Model(ins_att,ins_vel,ins_pos,ins_pos_xyz,dt)

%%% Wind Field %%%

% Velocity of Wind Field in ENU
[e,n,u] = wind_field(norm(ins_pos_xyz));
vel_w_enu = [e;n;u];

% Velocity of Wind Field in NED
enu2ned = [0 1 0;
           1 0 0;
           0 0 -1];
vel_w = enu2ned*vel_w_enu;

%%% Orientation %%%

% The original attitude in DCM from Euler representation
C_n_b = angle2dcm(ins_att(1),ins_att(2),ins_att(3),'XYZ');

% The second term is due to the rotation of the Earth with respect to an inertial frame

% So the skew-symmetric matrix is the following noting it is a function of latitude (Eq. 5.34)
omega_ie = 2.9924e-7; % rad/s
Omega_n_ie = omega_ie * [0               sin(ins_pos(1))  0;
                        -sin(ins_pos(1)) 0               -cos(ins_pos(1));
                         0               cos(ins_pos(1))  0];

% The third term arises from the rotation of the local-navigation-frame axes as the frame center moves with respect to the Earth.

% Parameters of Venus
e = 0;
R_0 = 6051800.0;

% The radius of curvature for north-south motion (Eq. 2.65)
R_N = R_0*(1-e^2)/(1-e^2*sin(ins_pos(1))^2)^(3/2);

% The radius of curvature for east-west motion (Eq. 2.66)
R_E = R_0/(1-e^2*sin(ins_pos(1))^2)^(1/2);

% The rotation rate vector (Eq. 5.37)
omega_n_en = [ins_vel(2)/(R_E+ins_pos(3));
             -ins_vel(1)/(R_N+ins_pos(3));
            (-ins_vel(2)*tan(ins_pos(1)))/(R_E+ins_pos(3))];

% The skew-symetric matrix alternative
Omega_n_en = [0            -omega_n_en(3) omega_n_en(2);
              omega_n_en(3) 0            -omega_n_en(1);
             -omega_n_en(2) omega_n_en(1) 0];

% Goal Attitude
r = vrrotvec([1;0;0],vel_w);
C_n_b_goal = vrrotvec2mat(r);

% Time derivative of attitute
C_n_b_dot = (C_n_b_goal-C_n_b)/dt;

% Calculate the first term, angular rate
Omega_b_ib = C_n_b'*(C_n_b_dot+(Omega_n_ie+Omega_n_en)*C_n_b);
omega_b_ib = [Omega_b_ib(3,2);
              Omega_b_ib(1,3);
              Omega_b_ib(2,1)];

% Cap angular rate to capabilities of aerobot
max_omega = 0.25;
omega_b_ib(omega_b_ib>max_omega) = max_omega;
omega_b_ib(omega_b_ib<-max_omega) = -max_omega;

%%% Acceleration %%%

% Acceleration due to gravity
g = gravity(norm(ins_pos_xyz));
acc_g = [0;0;g];

% The acceleration considering the specific force and centrifugal acceleration
acc_r = (Omega_n_en+2*Omega_n_ie)*ins_vel;

% Change in Velocity
delta_vel = vel_w-ins_vel;

% Kinetic Energy per unit mass
E = 1/2*delta_vel.^2;

% Force per unit mass or acceleration applied by wind
acc_w = E/dt;

% Cap acceleration of wind by g force experienced
if norm(acc_w) > 3*8.87
    acc_w = acc_w/norm(acc_w)*3*8.87;
end

% Adjust direction of acceleration based on sign
acc_w = sign(delta_vel).*acc_w;

% Total acceleration
acc_n = acc_w-acc_g+acc_r;

% Specific Force
f_ib_b = angle2dcm(ins_att(1),ins_att(2),ins_att(3),'XYZ')'*acc_n;

end
%% Development Information
% MAE 466 Spacecraft Dynamics 
% DCM_to_Euler_Cottrill.m
% 
% input[R,sequence]: R is a 3 x 3 rotation matrix (DCM)
%                    Sequence is number representing the Euler Angle sequence                
%
% output[theta]: Theta is a 3 x 1 tuple of arbitrary Euler angles
% 
% Assumptions:
% (1) None
% 
% 
% Primary Developer Contact Information:
% Heath Cottrill
% Student
% Statler College of Engineering & Mineral Resources
% Dept. Mechanical and Aerospace Engineering
% West Virginia University (WVU)
% hsc0009@mix.wvu.edu
%
%
%
% Development History
% Date              Developer        Comments
% ---------------   -------------    --------------------------------
% May 20, 2021      H. Cottrill      Initial implemention
%
%%

function [theta] = DCM_to_Euler_Cottrill(R,sequence)

% Create a zeros vector to house theat values
theta = zeros(3,1);

% An if statement to change command based on sequence
if sequence == 313
    theta(2) = acos(R(9));
    theta(1) = atan2(R(3),-R(6));
    theta(3) = atan2(R(7),R(8));
elseif sequence == 321
    theta(2) = asin(-R(7));
    theta(1) = atan2(R(4),R(1));
    theta(3) = atan2(R(8),R(9));
end


end

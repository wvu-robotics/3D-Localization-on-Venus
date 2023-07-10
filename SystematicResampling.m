function [x,w] = SystematicResampling(x,w)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% SystematicResampling.m
% 
% Purpose: Resampling for particle filter
% 
% Input: particles
%        weights
%
% Output: particles
%         weights
%
% Source: Li, T., Bolic, M., & Djuric, P. M. (2015). Resampling Methods for 
%         Particle Filtering: Classification, implementation, and 
%         strategies. IEEE Signal Processing Magazine, Signal Processing 
%         Magazine, IEEE, IEEE Signal Process. Mag, 32(3), 70–86. 
%         https://doi.org/10.1109/MSP.2014.2330626
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

% Preallocation
N = length(w);
Q = cumsum(w);
T = linspace(0,1-1/N,N) + rand(1)/N;
T(N+1) = 1;
indx = zeros(1,N);
i=1;
j=1;

% Perform systematic resampling
while (i<=N)
    if (T(i)<Q(j))
        indx(i)=j;
        i=i+1;
    else
        j=j+1;        
    end
end

% Update particles and weights based on Systematic resampling
x = x(:,indx);
w = ones(size(w))/N;

end
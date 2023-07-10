function [x,w] = ResidualResampling(x,w)

% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% ResidualResampling.m
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
M = length(w);
w = w / sum(w);
indx = zeros(1, N);

% Integer parts
Ns = floor(N.* w);
R = sum(Ns);

% Draw the deterministic part
i = 1;
j = 0;
while j < M
    j = j + 1;
  cnt = 1;
  while cnt <= Ns(j)
    indx(i) = j;
    i = i + 1; cnt = cnt + 1;
  end
end

% The fractions: Multinomial) resampling
N_rdn = N - R;
Ws =(N*w - Ns)/N_rdn;
Q = cumsum(Ws);
while(i <= N)
    sampl = rand; 
    j = 1;
    while(Q(j) < sampl)
        j = j + 1;
    end
    indx(i) = j;
    i = i + 1;
end

% Update particles and weights based on Systematic resampling
x = x(:,indx);
w = ones(size(w))/M;


%% Development Information
% WVU Interactive Robotics Laboratory 
% Venus Localization Simulator
% step_save_data.m
% 
% Purpose: Save Data at Each Time Step in Simulation
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
%%
    
% Topographic Localization Pose
mn.px(simulation.i) = pf.xf_1(1);
mn.py(simulation.i) = pf.xf_1(2);
mn.pz(simulation.i) = pf.xf_1(3);
mn.roll(simulation.i) = pf.xf_1(4);
mn.pitch(simulation.i) = pf.xf_1(5);
mn.yaw(simulation.i) = pf.xf_1(6);

% Pressure Localization Pose
pr.px(simulation.i) = pf.xf_2(1);
pr.py(simulation.i) = pf.xf_2(2);
pr.pz(simulation.i) = pf.xf_2(3);
pr.roll(simulation.i) = pf.xf_2(4);
pr.pitch(simulation.i) = pf.xf_2(5);
pr.yaw(simulation.i) = pf.xf_2(6);

% Dead Reckoning Error
errors.dr.px(simulation.i) = agent.tpx(simulation.i)- agent.px(simulation.i);
errors.dr.py(simulation.i) = agent.tpy(simulation.i) - agent.py(simulation.i);
errors.dr.pz(simulation.i) = agent.tpz(simulation.i) - agent.pz(simulation.i);
errors.dr.p(simulation.i) = sqrt(errors.dr.px(simulation.i)^2 + errors.dr.py(simulation.i)^2 + errors.dr.pz(simulation.i)^2);
errors.dr.roll(simulation.i) = agent.troll(simulation.i) - agent.roll(simulation.i);
errors.dr.pitch(simulation.i) = agent.tpitch(simulation.i) - agent.pitch(simulation.i);
errors.dr.yaw(simulation.i) = agent.tyaw(simulation.i) - agent.yaw(simulation.i);

% Topographic Localization Error
errors.mn.px(simulation.i) = agent.tpx(simulation.i) - mn.px(simulation.i);
errors.mn.py(simulation.i) = agent.tpy(simulation.i) - mn.py(simulation.i);
errors.mn.pz(simulation.i) = agent.tpz(simulation.i) - mn.pz(simulation.i);
errors.mn.p(simulation.i) = sqrt(errors.mn.px(simulation.i)^2 + errors.mn.py(simulation.i)^2 + errors.mn.pz(simulation.i)^2);
errors.mn.roll(simulation.i) = agent.troll(simulation.i) - mn.roll(simulation.i);
errors.mn.pitch(simulation.i) = agent.tpitch(simulation.i) - mn.pitch(simulation.i);
errors.mn.yaw(simulation.i) = agent.tyaw(simulation.i) - mn.yaw(simulation.i);

% Pressure Localization Error
errors.pr.px(simulation.i) = agent.tpx(simulation.i) - pr.px(simulation.i);
errors.pr.py(simulation.i) = agent.tpy(simulation.i) - pr.py(simulation.i);
errors.pr.pz(simulation.i) = agent.tpz(simulation.i) - pr.pz(simulation.i);
errors.pr.p(simulation.i) = sqrt(errors.pr.px(simulation.i)^2 + errors.pr.py(simulation.i)^2 + errors.pr.pz(simulation.i)^2);
errors.pr.roll(simulation.i) = agent.troll(simulation.i) - pr.roll(simulation.i);
errors.pr.pitch(simulation.i) = agent.tpitch(simulation.i) - pr.pitch(simulation.i);
errors.pr.yaw(simulation.i) = agent.tyaw(simulation.i) - pr.yaw(simulation.i);

% Residual
errors.resid_pf1(simulation.i) = errors.dr.p(simulation.i) - errors.mn.p(simulation.i);
errors.resid_pf2(simulation.i) = errors.dr.p(simulation.i) - errors.pr.p(simulation.i);

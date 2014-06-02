% Derive, simulate, plot, and animate one leg of ATRIAS
% Author: Andrew Peekema

% Cleanup
clc       % Clear the command prompt
clear all % Remove all workspace variables
close all % Close all figure windows

% Solve the kinematics
k = kinematicEqns;

% Solve the velocity kinematics
z = velocityEqns(k);

% Solve the dynamics
eqs = dynamicEqns(k,z);

% Add feedback linearization
eqs = feedbackLinearization(k,eqs);

% Convert the control torques to c code
tauA = ccode(eqs(1));
tauB = ccode(eqs(2));

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
%tauA = ccode(eqs(1));
%tauB = ccode(eqs(2));

% Convert the control torques to matlab functions
tauA = matlabFunction(eqs(1));
tauB = matlabFunction(eqs(2));

% Declare constants
g = 9.81; % m/s^2
%g = 0; % m/s^2
% Link 1 and 5
r1 = 0.5;  % length (m)
% Link 2 and 4
r2 = 0.5; % length (m)
% Mass of ATRIAS
m = 59.9; % kg
% Rotor inertia
I = 0.0019; % kg*m^2
% Motor inertia (as seen by the output)
I3 = 50^2*I;
I6 = I3;
% Motor damping (as seen by the output)
c3 = 19; % N*s/rad
c6 = c3;
% Spring parameters
ks = 1600; % N/rad
cS = 1.49; % N*s/rad

% Test the functions
FxDes = 0;
FyDes = 59.9*9.81;
k1_11 = 10;
k1_22 = 10;
k2_11 = k1_11^2;
k2_22 = k1_22^2;
dq1 = 0;
dq2 = 0;
dq3 = 0;
dq6 = 0;
%q1 = pi/4;
%q2 = pi/2;
%q3 = 3*pi/4;
%q6 = pi/4;
q1 = 1.066;
q2 = 1.044;
q3 = 2.11;
q6 = 1.066;
% Insert some error into the desired angles
%q3des = q3+0.1;
%q6des = q6-0.1;

% Torques for force control
testA = tauA(FxDes,FyDes,I3,c3,cS,dq1,dq2,dq3,dq6,g,k1_11,k2_11,ks,m,q1,q2,q3,q6,r1,r2)
testB = tauB(FxDes,FyDes,I6,c6,cS,dq1,dq2,dq3,dq6,g,k1_22,k2_22,ks,m,q1,q2,q3,q6,r1,r2)

% Torques for position control
%testA = tauA(I3,c3,cS,dq1,dq2,dq3,k1_11,k2_11,ks,q1,q2,q3,q3des)
%testB = tauB(I6,c6,cS,dq1,dq6,k1_22,k2_22,ks,q1,q6,q6des)

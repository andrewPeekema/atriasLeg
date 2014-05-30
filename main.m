% Derive, simulate, plot, and animate one leg of ATRIAS
% Author: Andrew Peekema

% Cleanup
clc       % Clear the command prompt
clear all % Remove all workspace variables
close all % Close all figure windows

display('Solving the equations of motion...')

% Solve the kinematics
k = kinematicEqns;

% Derive the output equation (y)
% Find the desired spring torque
syms FxDes FyDes m g ks q1 q2 q3 q6 r1 r2 real
% Virtual spring in the y direction
% TODO: Determine why FyDes doesn't seem to change the dynamics
Ff0 = [FxDes; FyDes; 0; 0; 0; 0];
Fh2 = k.h2f0.transAdj*Ff0;
Fh5 = k.h5f0.transAdj*Ff0;
tau_h2_des = Fh2(6);
tau_h5_des = Fh5(6);

% The actual spring torque
tau_h2 = ks*((q1+q2)-q3);
tau_h5 = ks*(q1-q6);

% Regulate the desired spring torque to the actual spring torque.  y will be
% driven to zero
y = [tau_h2 - tau_h2_des;
     tau_h5 - tau_h5_des];

%{
% Solve the velocity kinematics
z = velocityEqns(k);

% Solve the dynamics
eqs = dynamicEqns(k,z);

% Add feedback linearization
eqs = feedbackLinearization(k,eqs);

% Use PD control
%eqs = pdControl(eqs);

display('...equations of motion solved')



display('Simulating the dynamics...')

% Substute constants into the dynamic equations
[c ddq1 ddq2 ddq3 ddq6] = subConstants(eqs);

% Initial state conditions
X0 = [pi/4 ...   % q1 (rad)
      0 ...      % dq1 (rad/s)
      pi/2 ...   % q2
      0 ...      % dq2
      3*pi/4 ... % q3
      0 ...      % dq3
      pi/4 ...   % q6
      0];        % dq6

% Time vector (s)
t = [0:0.01:0.3];

% Integrate the time response of the system
sol = dynamicsSim(t,X0,ddq1,ddq2,ddq3,ddq6);

display('...dynamics simulated')



% Plot the response
q1 = sol.X(:,1);
q2 = sol.X(:,3);
plot(q1,q2,'.');
title('State Space Response')
xlabel('q1 (rad)')
ylabel('q2 (rad)')

% Animate the response
exportVideo = false;
animation(c,k,sol,exportVideo);
%}

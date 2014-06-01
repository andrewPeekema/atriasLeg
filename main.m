% Derive, simulate, plot, and animate one leg of ATRIAS
% Author: Andrew Peekema

% Cleanup
clc       % Clear the command prompt
clear all % Remove all workspace variables
close all % Close all figure windows

display('Solving the equations of motion...')

% Solve the kinematics
k = kinematicEqns;

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

% Derive, simulate, plot, and animate one leg of ATRIAS
% Author: Andrew Peekema

% Cleanup
clc       % Clear the command prompt
clear all % Remove all workspace variables
close all % Close all figure windows

tic
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
toc



tic
display('Simulating the dynamics...')

% Substute constants into the dynamic equations
[c eqs] = subConstants(eqs);

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
t = [0:0.01:3];

% Integrate the time response of the system
sol = dynamicsSim(t, X0, eqs.ddq1, eqs.ddq2, eqs.ddq3, eqs.ddq6);

display('...dynamics simulated')
toc



% Plot the response
plot(sol.t,sol.X(:,1),'b');
hold on
plot(sol.t,sol.X(:,3),'r');
plot(sol.t,sol.X(:,5),'g');
plot(sol.t,sol.X(:,7),'k');
title('State Space Response')
xlabel('Time (sec)')
ylabel('Angle (rad)')
legend('q1','q2','q3','q4','Location','Best')

% Find and plot the forces
[Fx Fy legLength] = stateToForces(k,sol.X);

figure
plot(legLength,hypot(Fx,Fy));

% Animate the response
exportVideo = false;
animation(c,k,sol,exportVideo);

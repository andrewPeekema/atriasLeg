function [c, ddq1, ddq2, ddq3, ddq6] = subConstants(eqs)
% Substitute constants into the dynamic equations
% Input
%   eqs: Dynamic equations (ddq1, ddq2, ddq3, ddq6)
% Output
%   c: struct of constants
%   ddq1: Angular acceleration of q1
%   ddq2: Angular acceleration of q2
%   ddq3: Angular acceleration of q3
%   ddq6: Angular acceleration of q6

% Declare constants
g = 9.81; % m/s^2
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

% PD control variables
kp = 10000;
kd = 1000;
q3des  = 3*pi/4;
dq3des = 0;
q6des  = pi/4;
dq6des = 0;

% Substitute constants into the dynamics
ddq1 = matlabFunction(subs(eqs.ddq1));
ddq2 = matlabFunction(subs(eqs.ddq2));
ddq3 = matlabFunction(subs(eqs.ddq3));
ddq6 = matlabFunction(subs(eqs.ddq6));

% Constants used for the animation
c.r1 = r1;
c.r2 = r2;

end % function

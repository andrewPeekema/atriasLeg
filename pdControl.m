function eqs = pdControl(eqs)
% Add feedback linearization on top of the equations of motion
% Input
%   eqs: equations of motion (ddq1, ddq2, ddq3, ddq6)
% Output
%   eqs: equations of motion with control added


% How does the torque enter the system?
syms I3 I6 real
g = [0    0
     0    0;
     0    0;
     0    0;
     0    0;
     1/I3 0;
     0    0;
     0    1/I6];

% PD control
syms kp kd q3 q6 dq3 dq6 q3des q6des dq3des dq6des real
u(1) = kp*(q3des - q3) + kd*(dq3des - dq3);
u(2) = kp*(q6des - q6) + kd*(dq6des - dq6);

% Put the control into the dynamic equations
eqs.ddq3 = eqs.ddq3 + g(6,1)*u(1);
eqs.ddq6 = eqs.ddq6 + g(8,2)*u(2);

end % function eqs

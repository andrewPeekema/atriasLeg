function eqs = feedbackLinearization(k,eqs)
% Add feedback linearization on top of the equations of motion
% Input
%   k: kinematic equations in SE3
%   eqs: equations of motion (ddq1, ddq2, ddq3, ddq6)
% Output
%   eqs: equations of motion with control added


% Define the natural dynamics (f)
syms q1 q2 q3 q6 dq1 dq2 dq3 dq6 q1des q2des q3des q6des real
f = [dq1;
     eqs.ddq1;
     dq2;
     eqs.ddq2
     dq3
     eqs.ddq3
     dq6
     eqs.ddq6];


%% Derive the output equation (y)
%% Find the desired spring torque
syms ks q1 q2 q3 q6 real

% The desired end effector force
syms FxDes FyDes real
% The wrench vector
Ff0 = [FxDes; FyDes; 0; 0; 0; 0];

% Move the desired force along one link
Ff2 = k.f2f0.transAdj*Ff0;
% Pin joint, remove torque on the free axis
Ff2(6) = 0;
% Move the forces up the next link
Fh2 = k.h2g2.transAdj*k.g2f2.transAdj*Ff2;
% The desired torque
tau_h2_des = Fh2(6);

% Repeat for the other member
% Move the desired force along one link
Ff5 = k.f5f0.transAdj*Ff0;
% Pin joint, remove torque on the free axis
Ff5(6) = 0;
% Move the forces up the next link
Fh5 = k.h5g5.transAdj*k.g5f5.transAdj*Ff5;
% The desired torque
tau_h5_des = Fh5(6);

% The actual spring torque
tau_h2 = ks*((q1+q2)-q3);
tau_h5 = ks*(q1-q6);

% Regulate the desired spring torque to the actual spring torque.  y will be
% driven to zero
y = [tau_h2 - tau_h2_des;
     tau_h5 - tau_h5_des];

% The state variables
q = {'q1' 'dq1' 'q2' 'dq2' 'q3' 'dq3' 'q6' 'dq6'};

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

% Gain matricies
syms k1_11 k1_22 k2_11 k2_22 real
k1 = [k1_11 0;
      0     k1_22];
k2 = [k2_11 0;
      0     k2_22];

% Take the lie derivatives necessary for the control
Lfy   = lieDerivative(y,f,q);
Lfy2  = lieDerivative(Lfy,f,q);
LgLfy = lieDerivative(Lfy,g,q);

% The control
u = simplify(-LgLfy\(Lfy2+k1*Lfy+k2*y));

% Return the control
eqs = u;

end % function eqs

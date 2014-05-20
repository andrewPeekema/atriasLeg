function sol = dynamicEqns(k,z)
% Generate the equations of motion
% Input
%   k: kinematic equations in SE3
%   z: velocity kinematics
% Output
%   sol: equations of motion

% Find the Lagrangian
syms m I3 I6 real

% Mass matrix of the third link
mm3 = m/2*eye(3,3);
Im3 = [0 0 0;
       0 0 0;
       0 0 I3];
M3 = [[mm3 zeros(3,3)];
      [zeros(3,3) Im3]];
% Mass matrix of the sixth link
mm6 = m/2*eye(3,3);
Im6 = [0 0 0;
       0 0 0;
       0 0 I6];
M6 = [[mm6 zeros(3,3)];
      [zeros(3,3) Im6]];

% Kinetic energy of the system
T = 1/2*z.g3'*M3*z.g3 + 1/2*z.g6'*M6*z.g6;

% Gravitational potential energy of the system
syms g real
V = g*(m/2*k.g3f0.y + m/2*k.g6f0.y);

% Add the spring potential energy
syms ks q1 q2 q3 q6 real
V = V + 1/2*ks*(q3-(q1+q2))^2 + 1/2*ks*(q6-q1)^2;

% The Lagrangian
L = T - V;

% The Euler-Lagrange equation
eqs = eulerLagrange(L,{'q1' 'q2' 'q3' 'q6'});

% Add motor damping
syms dq3 dq6 c3 c6 real
eqs(3) = eqs(3) + dq3*c3;
eqs(4) = eqs(4) + dq6*c6;

% Add spring damping
% Find the virtual work (F*d), and take the derivative w.r.t. each of the variables
syms dq1 dq2 cS real
virtualWork = (dq6-dq1)*cS*(q6-q1) + (dq3-dq1-dq2)*cS*(q3-q1-q2);
eqs(1) = eqs(1) + diff(virtualWork,q1);
eqs(2) = eqs(2) + diff(virtualWork,q2);
eqs(3) = eqs(3) + diff(virtualWork,q3);
eqs(4) = eqs(4) + diff(virtualWork,q6);

% Solve for acceleration
syms ddq1 ddq2 ddq3 ddq6 real
sol = solve(eqs,ddq1,ddq2,ddq3,ddq6);
sol.ddq1 = simplify(sol.ddq1);
sol.ddq2 = simplify(sol.ddq2);
sol.ddq3 = simplify(sol.ddq3);
sol.ddq6 = simplify(sol.ddq6);

end % dynamicEqns

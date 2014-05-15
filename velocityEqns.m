function z = velocityEqns(k)
% Generate the velocity kinematics for one leg of ATRIAS
% Input
%   k: kinematic equations in SE3
% Output
%   z: velocity kinematics

syms dq1 dq2 dq3 dq4 real

% The toe is on the ground
z.f0 = zeros(6,1);

% The first link
z.f1 = k.f1f0.invAdj*(z.f0+[zeros(5,1); dq1]);
z.g1 = k.g1f1.invAdj*z.f1;
z.h1 = k.h1g1.invAdj*z.g1;

% The second link
z.f2 = k.f2h1.invAdj*(z.h1+[zeros(5,1); dq2]);
z.g2 = k.g2f2.invAdj*z.f2;
z.h2 = k.h2g2.invAdj*z.g2;

% The third link (motor)
z.g3 = k.g3h2.invAdj*(z.h2+[zeros(5,1); dq3]);

% The fourth link
z.f4 = k.f4f1.invAdj*(z.f1+[zeros(5,1); dq2]);
z.g4 = k.g4f4.invAdj*z.f4;
z.h4 = k.h4g4.invAdj*z.g4;

% The fifth link
z.f5 = k.f5h4.invAdj*(z.h4+[zeros(5,1); -dq2]);
z.g5 = k.g5f5.invAdj*z.f5;
z.h5 = k.h5g5.invAdj*z.g5;

% The sixth link (motor)
z.g6 = k.g6h5.invAdj*(z.h5+[zeros(5,1); dq4]);

end % velocityEqns

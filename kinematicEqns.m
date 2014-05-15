function k = kinematicEqns
% Output
%   k: kinematics of one leg of ATRIAS

syms q1 r1 q2 r2 q3 q4 real

%% The base
k.f0 = SE3;


%% First link
k.f1f0 = k.f0*SE3([0 0 0 0 0 q1]);

% Center
k.g1f1 = SE3([r1/2 0 0]);
k.g1f0 = k.f1f0*k.g1f1;

% End
k.h1g1 = SE3([r1/2 0 0]);
k.h1f0 = k.g1f0*k.h1g1;


%% Second link
k.f2h1 = SE3([0 0 0 0 0 q2]);
k.f2f0 = k.h1f0*k.f2h1;
k.f2f0.g = simplify(k.f2f0.g,'IgnoreAnalyticConstraints',true); % Clean up notation

% Center
k.g2f2 = SE3([r2/2 0 0]);
k.g2f0 = k.f2f0*k.g2f2;

% End
k.h2g2 = SE3([r2/2 0 0]);
k.h2f0 = k.g2f0*k.h2g2;


%% Third link (motor center)
k.g3h2 = SE3([0 0 0 0 0 q3]);
k.g3f0 = k.h2f0*k.g3h2;
k.g3f0.g = simplify(k.g3f0.g,'IgnoreAnalyticConstraints',true); % Clean up notation


%% Fourth link
k.f4f1 = SE3([0 0 0 0 0 q2]);
k.f4f0 = k.f1f0*k.f4f1;
k.f4f0.g = simplify(k.f4f0.g,'IgnoreAnalyticConstraints',true); % Clean up notation

% Center
k.g4f4 = SE3([r2/2 0 0]);
k.g4f0 = k.f4f0*k.g4f4;

% End
k.h4g4 = SE3([r2/2 0 0]);
k.h4f0 = k.g4f0*k.h4g4;


%% Fifth link
k.f5h4 = SE3([0 0 0 0 0 -q2]);
k.f5f0 = k.h4f0*k.f5h4;
k.f5f0.g = simplify(k.f5f0.g,'IgnoreAnalyticConstraints',true); % Clean up notation

% Center
k.g5f5 = SE3([r1/2 0 0]);
k.g5f0 = k.f5f0*k.g5f5;

% End
k.h5g5 = SE3([r1/2 0 0]);
k.h5f0 = k.g5f0*k.h5g5;


%% Sixth link (motor center)
k.g6h5 = SE3([0 0 0 0 0 q4]);
k.g6f0 = k.h5f0*k.g6h5;
k.g6f0.g = simplify(k.g6f0.g,'IgnoreAnalyticConstraints',true); % Clean up notation


end % kinematicEqns

function [Fx,Fy,legLength] = stateToForces(k,X)
% Given
%   k: kinematic equations in SE3
%   X: The state vector over time
% Return
%   Fx = Forces in the horizontal direction
%   Fy = Forces in the vertical direction
%   legLength = Leg length over time

% Find the workspace wrench in terms of state variables
syms Fx Fy ks cS q1 q2 q3 q6 dq1 dq2 dq3 dq6 real

% The workspace wrench
Ff0 = [Fx; Fy; 0; 0; 0; 0];

% Move the wrench along one link
Ff2 = k.f2f0.transAdj*Ff0;
% Pin joint, remove torque on the free axis
Ff2(6) = 0;
% Move the forces up the next link
Fh2 = k.h2g2.transAdj*k.g2f2.transAdj*Ff2;
% The torque
tau_h2 = Fh2(6);

% Repeat for the other member
% Move the wrench along one link
Ff5 = k.f5f0.transAdj*Ff0;
% Pin joint, remove torque on the free axis
Ff5(6) = 0;
% Move the forces up the next link
Fh5 = k.h5g5.transAdj*k.g5f5.transAdj*Ff5;
% The torque
tau_h5 = Fh5(6);

% Solve for Fx and Fy in terms of torque
syms Tau_h2 Tau_h5 real
FxFy = solve(tau_h2-Tau_h2, tau_h5-Tau_h5, Fx, Fy);
Fx_given_tau = simplify(FxFy.Fx);
Fy_given_tau = simplify(FxFy.Fy);

% The actual spring torque
Tau_h2 = ks*((q1+q2)-q3);
Tau_h5 = ks*(q1-q6);

% Substitute spring torque into Fx and Fy
eqs.Fx_given_q = simplify(subs(Fx_given_tau, {'Tau_h2','Tau_h5'}, {Tau_h2,Tau_h5}));
eqs.Fy_given_q = simplify(subs(Fy_given_tau, {'Tau_h2','Tau_h5'}, {Tau_h2,Tau_h5}));

% The leg length equation
eqs.legLength = simplify(k.h2f0.distance);

% Substitute in constants
[c eqs] = subConstants(eqs);
Fx_given_q = eqs.Fx_given_q;
Fy_given_q = eqs.Fy_given_q;
legLength_fxn = eqs.legLength;

% Substitute in the state
Q1 = X(:,1);
Q2 = X(:,3);
Q3 = X(:,5);
Q6 = X(:,7);
dQ1 = X(:,2);
dQ2 = X(:,4);
dQ3 = X(:,6);
dQ6 = X(:,8);

% For each element
Fx = NaN(1,length(Q1));
Fy = NaN(1,length(Q1));
legLength = NaN(1,length(Q1));
parfor I = 1:length(Q1)
    Fx(I) = Fx_given_q(Q1(I), Q2(I), Q3(I), Q6(I));
    Fy(I) = Fy_given_q(Q1(I), Q2(I), Q3(I), Q6(I));
    legLength(I) = legLength_fxn(Q2(I));
end

end % function stateToForces

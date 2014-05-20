function sol = dynamicsSim(t,X0,ddq1,ddq2,ddq3,ddq6)
% Simulates the time response of one leg of ATRIAS
% Given
%   t: Time vector to integrate over
%   X0: Initial state [q1 dq1 q2 dq2 q3 dq3 q6 dq6]
%   ddq1: Angular acceleration for q1
%   ddq2: Angular acceleration for q2
%   ddq3: Angular acceleration for q3
%   ddq6: Angular acceleration for q6
% Returns
%   sol.t: time vector
%   sol.X: state matrix
% Author: Andrew Peekema

% Simulation tolerances
options = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9);

% Simulate the dynamics over a time interval
[sol.t sol.X] = ode45(@dynamics,t, X0, options);

function dX = dynamics(t,X)
    % t == time
    % X == the state
    q1  = X(1); % Angular Position
    dq1 = X(2); % Angular Velocity
    q2  = X(3);
    dq2 = X(4);
    q3  = X(5);
    dq3 = X(6);
    q6  = X(7);
    dq6 = X(8);

    % Return the state derivative
    dX = zeros(8,1);
    % Link 1
    dX(1) = dq1;
    dX(2) = ddq1(dq1,dq2,dq3,dq6,q1,q2,q3,q6);
    % Link 2
    dX(3) = dq2;
    dX(4) = ddq2(dq1,dq2,dq3,dq6,q1,q2,q3,q6);
    % Link 3 (motor)
    dX(5) = dq3;
    dX(6) = ddq3(dq1,dq2,dq3,q1,q2,q3);
    % Link 6 (motor)
    dX(7) = dq6;
    dX(8) = ddq6(dq1,dq6,q1,q6);

    %{
    % For testing with fixed motors
    % Link 3 (motor)
    dX(5) = X0(6);
    dX(6) = 0;
    % Link 6 (motor)
    dX(7) = X0(8);
    dX(8) = 0;
    %}
end % dynamics

end % dynamicsSim

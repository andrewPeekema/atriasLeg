function animation(c,k,sol,exportVideo)
% 3D Visualization Template
% Input
%   c: Simulation constants
%   sol: Simulation solution
%   exportVideo: Should the video be exported? (True/False)
% Output
%   An animation
% By Andrew Peekema

% Unpack constants
r1 = c.r1;
r2 = c.r2;

% Create visualization objects
link1 = CubeClass([r1 0.1 0.1]);
link2 = CubeClass([r2 0.1 0.1]);
link3 = CubeClass([r2 0.1 0.1]);
link4 = CubeClass([r2 0.1 0.1]);
link5 = CubeClass([r1 0.1 0.1]);
link6 = CubeClass([r1 0.1 0.1]);

% Create link transformations
from = {'r1' 'r2'};
to   = {r1 r2};
g1f0 = matlabFunction(subs(k.g1f0.g,from,to));
g2f0 = matlabFunction(subs(k.g2f0.g,from,to));
g3f0 = matlabFunction(subs(k.g3f0.g,from,to));
g4f0 = matlabFunction(subs(k.g4f0.g,from,to));
g5f0 = matlabFunction(subs(k.g5f0.g,from,to));
g6f0 = matlabFunction(subs(k.g6f0.g,from,to));

% Create a figure handle
h.figure = figure;

% Put the shapes into a plot
link1.plot
link2.plot
link3.plot
link4.plot
link5.plot
link6.plot

% Figure properties
view(2)
title('Simulation')
xlabel('x Position (m)')
ylabel('y Position (m)')
zlabel('z Position (m)')
% Set axis limits
aLim = (r1+r2+r1)*1.1;
axis([-aLim aLim ... % x
      -aLim aLim ... % y
      -1.0 1.0]);  % z

% Speed up if watching in realtime
if exportVideo
    frameStep = 3;
else
    frameStep = 4;
end

% Iterate over state data
for it = 1:frameStep:length(sol.t)
    q1 = sol.X(it,1);
    q2 = sol.X(it,3);
    q3 = sol.X(it,5);
    q6 = sol.X(it,7);

    % Link positions
    link1.resetFrame
    link1.globalMove(SE3(g1f0(q1)))
    link2.resetFrame
    link2.globalMove(SE3(g2f0(q1,q2)))
    link3.resetFrame
    link3.globalMove(SE3(g3f0(q1,q2,q3)))
    link4.resetFrame
    link4.globalMove(SE3(g4f0(q1,q2)))
    link5.resetFrame
    link5.globalMove(SE3(g5f0(q1,q2)))
    link6.resetFrame
    link6.globalMove(SE3(g6f0(q1,q2,q6)))

    % Update data
    link1.updatePlotData
    link2.updatePlotData
    link3.updatePlotData
    link4.updatePlotData
    link5.updatePlotData
    link6.updatePlotData

    % Draw figure
    drawnow

    % Save the frames
    if exportVideo
        frame = getframe(h.figure);
        imwrite(frame.cdata, sprintf('./video/%04d.png',it));
    end
end % for it = ...

end % animation

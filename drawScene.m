function drawScene( robot, box )
%DRAWSCENE Summary of this function goes here
%   Detailed explanation goes here

% figure
clf
xlim([-5 5])
ylim([0 10])

robot.draw_bot();
hold on
box.draw();
end


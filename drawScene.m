function drawScene( robot, box )
%DRAWSCENE Summary of this function goes here
%   Detailed explanation goes here

figure
robot.draw_bot();
hold on
box.draw();
end


%% Test out new orientation etc for the box implementation

clear

figure

new_box = Box();
thePause = .1;

% rotate the box
orientations = 360;
angle_step = 1;
trans = 0;
for i = 1 : orientations
    new_box = trans_rot(new_box, [0;0], angle_step);    
    clf
    xlim([-5 5])
    ylim([0 10])
    new_box.draw();
    pause(thePause);
end
%% Test out new orientation etc for the box implementation

clear

figure

new_box = Box();
thePause = 1.;

% rotate the box
orientations = [0:1:360];
trans = 0;
for i = 1:numel(orientations)
    new_box = trans_rot_box(new_box, [0;0], orientations(i));    
    clf
    xlim([-5 5])
    ylim([0 10])
    new_box.draw();
    pause(thePause);
end
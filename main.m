%% Simulates the experiment
clear
thePause = .001;
the_rob = Robot();
boxer = Box();
% drawScene(the_rob, boxer);
pick_states = the_rob.pick_up(boxer);
figure
for i=1:size(pick_states,1)
    the_rob = change_state(the_rob, pick_states(i,:));
    drawScene(the_rob, boxer);
    pause(thePause);
end

[rob_moving, box_moving] = the_rob.move_box(boxer);

for i=1:size(rob_moving,1)
    the_rob = change_state(the_rob, rob_moving(i,:));
    boxer = move_by_grasp(boxer, box_moving(i,:));
    drawScene(the_rob, boxer);
    pause(thePause);
end
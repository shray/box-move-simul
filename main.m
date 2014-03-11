%% Simulates the experiment
clear
the_rob = Robot();
boxer = Box();
drawScene(the_rob, boxer);
pick_states = the_rob.pick_up(boxer);

for i=1:size(pick_states,1)
    the_rob = change_state(the_rob, pick_states(i,:));
    drawScene(the_rob, boxer);
    pause(.1);
end


%% Simulates the experiment
clear
the_rob = Robot();
boxer = Box();
% drawScene(the_rob, boxer);
a = the_rob.pick_up(boxer);
the_rob = change_state(the_rob, a(1,:));
drawScene(the_rob, boxer);
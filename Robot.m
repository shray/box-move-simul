classdef Robot
    % contains state of the robot and methods to move it around
    properties%(GetAccess=private)
        lens = [1 1 .5 .5]; % [back-left back-right fore-left fore-right] 
        state % angles - degrees
        base = [0;0]; % assume base at origin
        elbows
        end_effs
    end
    methods
        
        function obj = Robot(init_state)
            %initialize state
            if nargin < 1
                obj.state = [60 120 120 360-120];                
            else
                obj.state = init_state;
            end            
            
            obj = update_params(obj);
        end
        
        function obj = update_params(obj)
            % updates variables for elbows, end effectors
            lt_elbow = obj.base + [cosd(obj.state(1))*obj.lens(1); sind(obj.state(1))*obj.lens(1)];
            rt_elbow = obj.base + [cosd(obj.state(2))*obj.lens(2); sind(obj.state(2))*obj.lens(2)];
            obj.elbows = [lt_elbow, rt_elbow];
            lt_ee = lt_elbow + [cosd(180 - (obj.state(3)-obj.state(1)))*obj.lens(3); sind(180 - (obj.state(3)-obj.state(1)))*obj.lens(3)];
            rt_ee = rt_elbow + [cosd(180 - (obj.state(4)-obj.state(2)))*obj.lens(4); sind(180 - (obj.state(4)-obj.state(2)))*obj.lens(4)];
            obj.end_effs = [lt_ee, rt_ee];
        end
        
        function states = pick_up(obj, box)
            % return states of the robot to pickup box
            init_state = obj.state;
            final_state = obj.get_state(box.get_grasp());            
            
        end
        
        function states = move_box(obj, box)
            % return 2D position of end-effectors after putting down box
            states = 0;
        end
        
        function draw_bot(obj)
            lt_elbow = obj.elbows(:,1);
            rt_elbow = obj.elbows(:,2);
            lt_ee = obj.end_effs(:,1);
            rt_ee = obj.end_effs(:,2);
            
            %draw
            hold on
            line([[obj.base(1); lt_elbow(1)], [obj.base(1); rt_elbow(1)], [lt_elbow(1); lt_ee(1)], [rt_elbow(1); rt_ee(1)]],...
                [[obj.base(2); lt_elbow(2)], [obj.base(2); rt_elbow(2)], [lt_elbow(2); lt_ee(2)], [rt_elbow(2); rt_ee(2)]])
            hold off
            
        end
        
        function grasp = get_state(obj, ee_locs)
            
        end
    end
        
end
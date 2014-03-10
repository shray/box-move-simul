classdef Robot
    % contains state of the robot and methods to move it around
    properties%(GetAccess=private)
        lens = 4*[1 1 .5 .5]; % [back-left back-right fore-left fore-right] 
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
            %angles = [30 75];
            final_state = obj.get_state(box.get_grasp());%, angles);
            % debug
            if numel(final_state) < 2
                obj.state(1) = angles(1);
                obj.state(2) = angles(2);
                obj = update_params(obj);
                obj.draw_bot();
            end
            states = final_state;            
        end
        
        function obj = change_state(obj, new_state)
            obj.state = new_state;
            obj = update_params(obj);
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
        
        function grasp = get_state(obj, ee_loc)
            % get final state for a certain end effector location and
            % assuming base at origin and fore-arm half the size of
                        
            % left back-arm
            a = ee_loc(1,1);
            b = ee_loc(2,1);
            l = obj.lens(3);
            c = (3*l^2+a^2+b^2)/(2*b);
            d = a/(2*b);
            capD = (4*d^2*c^2 - 4*(1+d^2)*(c^2-4*l^2));
            x = 2*(d*c)/(2*(1+d^2)) * [sqrt(capD) -sqrt(capD)];
            y = c - d*x;
            
            elbow_angles = to_degrees([atan2(y(1),x(1)); atan2(y(2),x(2))]);
            lt_elbow = [x; y];
            
            % right back-arm
            a = ee_loc(1,2);
            b = ee_loc(2,2);
            l = obj.lens(3);
            c = (3*l^2+a^2+b^2)/(2*b);
            d = a/(2*b);
            capD = (4*d^2*c^2 - 4*(1+d^2)*(c^2-4*l^2));
            x = 2*(d*c)/(2*(1+d^2)) * [sqrt(capD) -sqrt(capD)];
            y = c - d*x;

            elbow_angles = [elbow_angles, to_degrees([atan2(y(1),x(1)); atan2(y(2),x(2))])];
            rt_elbow = [x; y];
            
%             lt_elbow = obj.base + [cosd(elbow_angles(1,1))*obj.lens(1); sind(elbow_angles(1,1))*obj.lens(1)];
%             rt_elbow = obj.base + [cosd(elbow_angles(2,1))*obj.lens(2); sind(elbow_angles(2,1))*obj.lens(2)];
            
%             % check if end-effector loc can be reached
%             if (norm(lt_ee-lt_elbow)>obj.lens(3) || norm(rt_ee-rt_elbow)>obj.lens(4))
%                 grasp = 0;
%                 return
%             end

            lt_ee = repmat([ee_loc(1,1); ee_loc(2,1)],1,2);
            rt_ee = repmat([ee_loc(1,2); ee_loc(2,2)],1,2);
            
            % angle b/w horizontal and fore-arms
            alpha_lt = (to_degrees(atan2((lt_ee(2,:)-lt_elbow(2,:)), (lt_ee(1,:)-lt_elbow(1,:)))))';
            alpha_rt = (to_degrees(atan2((rt_ee(2,:)-rt_elbow(2,:)), (rt_ee(1,:)-rt_elbow(1,:)))))';
            grasp = [elbow_angles(:,1) elbow_angles(:,2) alpha_lt+elbow_angles(:,1) 360 - (alpha_rt+elbow_angles(:,2))];
            
        end
    end
        
end
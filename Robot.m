classdef Robot
    % contains state of the robot and methods to move it around
    properties%(GetAccess=private)
        lens = 4*[1. 1. .3 .3]; % [back-left back-right fore-left fore-right] 
        state % angles - degrees
        base = [0;0]; % assume base at origin
        elbows
        end_effs
        state_speed = [1 1 1 1]; % currently specified max speed
    end
    methods
        
        function obj = Robot(init_state)
            %initialize state
            if nargin < 1
                obj.state = [120 60 360-120 120];                
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
        
        function ee = compute_ee(obj, state)
            %return the end-effector positions for a given robot state
            lt_elbow = obj.base + [cosd(state(1))*obj.lens(1); sind(state(1))*obj.lens(1)];
            rt_elbow = obj.base + [cosd(state(2))*obj.lens(2); sind(state(2))*obj.lens(2)];
            obj.elbows = [lt_elbow, rt_elbow];
            lt_ee = lt_elbow + [cosd(180 - (state(3)-state(1)))*obj.lens(3); sind(180 - (state(3)-state(1)))*obj.lens(3)];
            rt_ee = rt_elbow + [cosd(180 - (state(4)-state(2)))*obj.lens(4); sind(180 - (state(4)-state(2)))*obj.lens(4)];
            ee = [lt_ee rt_ee];
        end
        
        function states = pick_up(obj, box)
            % return states of the robot to pickup box
            init_state = obj.state;
            %angles = [30 75];
            final_state = obj.get_state(box.get_grasp());%, angles);
            % debug
            if numel(final_state) < 2
                'Box can not be reached'
            end
            
            % generate states
            num_states = ceil(max(abs(final_state-init_state)./obj.state_speed));
            step_vector = (final_state-init_state)/num_states;
            states = [];
            
            for i = 0:num_states
                states = [states; init_state+i*step_vector];
            end
        end
        
        function obj = change_state(obj, new_state)
            obj.state = new_state;
            obj = update_params(obj);
        end
        
        function [states, box_disp] = move_box(obj, box)
            %TODO: check if end-effectors are grasping the box
            % return states of the bot while moving the box
            % of end-effectors after putting down box and box 
            % displacements
            
            init_state = obj.state;
            b_angle_change = 180 - 2*init_state(2);
            final_state = obj.state + [b_angle_change b_angle_change 0 0];
            %generate states
            num_states = ceil(max(abs(final_state-init_state)./obj.state_speed));
            step_vector = (final_state-init_state)/num_states;
            states = [];
            box_disp=[];
            prev_state = init_state;
            
            for i = 0:num_states
                next_state = init_state+i*step_vector;
                states = [states; next_state];
                prev_ee = obj.compute_ee(prev_state);
                next_ee = obj.compute_ee(next_state);
                box_disp = [box_disp; [next_ee(:,1)-prev_ee(:,1)]'];
                prev_state = next_state;
            end
        end
        
        function draw_bot(obj)
            lt_elbow = obj.elbows(:,1);
            rt_elbow = obj.elbows(:,2);
            lt_ee = obj.end_effs(:,1);
            rt_ee = obj.end_effs(:,2);
            
            %draw
            hold on
            line([[obj.base(1); lt_elbow(1)], [lt_elbow(1); lt_ee(1)]],...
                [[obj.base(2); lt_elbow(2)], [lt_elbow(2); lt_ee(2)]], 'color', 'r')
            line([[obj.base(1); rt_elbow(1)], [rt_elbow(1); rt_ee(1)]],...
                [[obj.base(2); rt_elbow(2)], [rt_elbow(2); rt_ee(2)]], 'color','b')
            scatter(obj.end_effs(1,:), obj.end_effs(2,:));
            hold off
            
        end
                
        
        function grasp = get_state(obj, ee_loc)
            % get final state for a certain end effector location and
            % assuming base at origin and fore-arm half the size of
                        
            % left back-arm
            a = ee_loc(1,1);
            b = ee_loc(2,1);
            l1 = obj.lens(3);
            l2 = obj.lens(1);
            c = (l2^2-l1^2+a^2+b^2)/(2*b);
            d = -a/b;
            capD = sqrt((4*d^2*c^2 - 4*(1+d^2)*(c^2-l2^2)));
            x = (-2*(d*c) + [capD -capD])./(2*(1+d^2));
            y = c + d*x;
            
            lt_elbow_a = to_degrees([atan2(y(1),x(1)); atan2(y(2),x(2))]);
            lt_ee = repmat([ee_loc(1,1); ee_loc(2,1)],1,2);

            lt_elbow = [x; y];
            
            alpha_lt = (to_degrees(atan2((lt_ee(2,:)-lt_elbow(2,:)), -(lt_ee(1,:)-lt_elbow(1,:)))))';
            
            % right back-arm
            a = ee_loc(1,2);
            b = ee_loc(2,2);
            l1 = obj.lens(4);
            l2 = obj.lens(2);
            c = (l2^2-l1^2+a^2+b^2)/(2*b);
            d = -a/b;
            capD = sqrt((4*d^2*c^2 - 4*(1+d^2)*(c^2-l2^2)));
            x = (-2*(d*c) + [capD -capD])./(2*(1+d^2));
            y = c + d*x;

            rt_elbow_a = to_degrees([atan2(y(1),x(1)); atan2(y(2),x(2))]);
            rt_elbow = [x; y];
            
            
%             lt_elbow = obj.base + [cosd(elbow_angles(1,1))*obj.lens(1); sind(elbow_angles(1,1))*obj.lens(1)];
%             rt_elbow = obj.base + [cosd(elbow_angles(2,1))*obj.lens(2); sind(elbow_angles(2,1))*obj.lens(2)];
            
%             % check if end-effector loc can be reached
%             if (norm(lt_ee-lt_elbow)>obj.lens(3) || norm(rt_ee-rt_elbow)>obj.lens(4))
%                 grasp = 0;
%                 return
%             end

            rt_ee = repmat([ee_loc(1,2); ee_loc(2,2)],1,2);
            
            % angle b/w horizontal and fore-arms
            
            alpha_rt = (to_degrees(atan2((rt_ee(2,:)-rt_elbow(2,:)), -(rt_ee(1,:)-rt_elbow(1,:)))))';                        
            
            rt_ee_a = (alpha_rt+rt_elbow_a);
            lt_ee_a = (alpha_lt+lt_elbow_a);
            
            angle_choices = obj.choose_bw_angles(lt_elbow_a, lt_ee_a, rt_elbow_a, rt_ee_a);
%             grasp = [elbow_angles(:,1) elbow_angles(:,2) alpha_lt+elbow_angles(:,1) 360 - (alpha_rt+elbow_angles(:,2))];
            %grasp = [lt_elbow_a(2) rt_elbow_a(1) alpha_lt(2)+lt_elbow_a(2) (alpha_rt(1)+rt_elbow_a(1))];            
            grasp = [lt_elbow_a(angle_choices(1)) rt_elbow_a(angle_choices(2)) lt_ee_a(angle_choices(1)) rt_ee_a(angle_choices(2))];            

        end
        
        function choices = choose_bw_angles(obj, l_el, r_el, l_ee, r_ee)
            % Two choices are presented for the right and left arms each to
            % reach a certain goal. We check constraints and return the
            % choice [left, right] if the constraints are met
            elbow_angle = [30 180]; % min, max
            base_angle = [30 120];
            for i = 1:size(l_el,1)
                for j = 1:size(r_el,1)
                    if (((l_ee(i))>elbow_angle(1)) && ((l_ee(i))<elbow_angle(2)))
                        %'reached2'
                        if (((r_ee(j))>elbow_angle(1)) && ((r_ee(j))<elbow_angle(2)))
                            %'reached3'
                            if ((l_el(i)-r_el(j))>base_angle(1) && (l_el(i)-r_el(j))<base_angle(2))
                                choices = [i,j];
                                return
                            end
                        end
                    end
                end                
            end
            choices = [2 1];
        end
                                            
    end
        
end
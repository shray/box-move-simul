classdef Box
    %BOX: class for box objects 
    %TODO: do not assume the box to be axis-aligned
    properties
        dimensions % height, width of box
        loc % x,y location of upper left corner of rectangle
        orientation % degrees of the upper side with horizontal (a-clockwise)
        grasp_locs % two points
        end_pts % [u-l b-l u-r b-r]
    end
    
    methods

        function obj = Box()
        % constructor - TODO check if the box reachable
            obj.loc = [3 2]';
            obj.dimensions = 4*[.25; .25];   
            obj.orientation = 0;
            obj = update_end_pts(obj);
        end

        function obj = update_end_pts(obj)
        % updates the box end points according to the loc,
        % dimensions and the orientation - which are expected to be
        % current
            u_l = obj.loc;
            u_r = u_l + obj.dimensions(2) * [ cosd(obj.orientation); ...
                                sind(obj.orientation)];
            b_l = u_l + obj.dimensions(1) * [ sind(obj.orientation); ...
                                -cosd(obj.orientation)];
            b_r = u_r + obj.dimensions(1) * [ sind(obj.orientation); ...
                                -cosd(obj.orientation)];
            obj.end_pts = [u_l b_l u_r b_r]; % set box end-pts
        end
        
        function grasp_locs = get_grasp(obj)
        % returns the mid-points of the parallel edges
% $$$             grasp_locs = [obj.loc(1), obj.loc(1)+obj.dimensions(1);...
% $$$                           (2*obj.loc(2)+obj.dimensions(2))/2, (2*obj.loc(2)+obj.dimensions(2))/2];
            ul = obj.end_pts(:,1); bl = obj.end_pts(:,2);
            ur = obj.end_pts(:,3); br = obj.end_pts(:,4);
            
            grasp_locs = [(ul+bl)/2 (ur+br)/2]; % mid point of the 'vertical' edges
            obj.grasp_locs = grasp_locs;
        end
        
        function draw(obj)
% $$$             rectangle('Position',[obj.loc(1) obj.loc(2) obj.dimensions(1) ...
% $$$                                 obj.dimensions(2)])
            ul = obj.end_pts(:,1); bl = obj.end_pts(:,2);
            ur = obj.end_pts(:,3); br = obj.end_pts(:,4);
            line([ [ul(1); ur(1)] [ul(1); bl(1)] [br(1); ur(1)] ...
                   [bl(1); br(1) ]], [[ul(2); ur(2)] [ul(2); bl(2)] ...
                   [br(2); ur(2)] [bl(2); br(2)] ] )
        end
        
        function obj = set_loc(obj, new_loc, new_orientation)
        % set new location for upper-left corner
            obj.loc = new_loc;

            if (nargin > 2)
                obj.orientation = new_orientation;
            end
            obj = update_end_pts(obj);                
        end
        
        function obj = trans_rot(obj, trans, rot)
        % translate and rotate box
            if nargin < 3
                rot = 0;
            end
            
            obj.loc = obj.loc + trans;
            obj.orientation = obj.orientation + rot;
            
            % 0 < orientation < 360
            while obj.orientation < 0
                obj.orientation = obj.orientation + 360;
            end
            while obj.orientation > 360
                obj.orientation = obj.orientation - 360;
            end
            
            obj = update_end_pts(obj);
        end
        
        function obj = move_by_grasp(obj, new_grasp)
        % Moves the box acc to new grasp-positions, assuming no
        % slippage for now
            gl = new_grasp(1:2)'; % left grasp
            gr = new_grasp(3:4)' % right grasp
            gvec = gr-gl/norm(gr-gl);
            obj.orientation = to_degrees(real(acos(dot(gvec, [1;0]))));
            %debug
            obj.orientation
            obj = update_end_pts(obj);
            old_g = obj.get_grasp();
            trans_l = gl - old_g(:,1); % compute translation as the
                                       % difference b/w new grasp
                                       % and the grasp pts of
                                       % rotated box
            trans_r = gr - old_g(:,2);
            if norm(trans_l-trans_r) > 1.e-10 % ensure trans the same
                %debug
                'this failed'
            end
            trans = (trans_l+ trans_r)/2;
            obj = trans_rot(obj, trans); % actually translate and rotate
        end
                
% $$$         function obj = displace(obj, vec)
% $$$             obj.loc = obj.loc + vec;
% $$$         end
    end
    
end


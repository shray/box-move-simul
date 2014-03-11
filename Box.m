classdef Box
    %BOX: class for box objects 
    
    properties
        dimensions % height, width of box
        loc % x,y location of upper left corner of rectangle
    end
    
    methods
        function obj = Box()
            % constructor - TODO check if the box reachable
            obj.loc = [3 2];
            obj.dimensions = 4*[.25; .25];
        end
            
        function grasp_locs = get_grasp(obj)
            % returns the mid-points of the parallel edges
            grasp_locs = [obj.loc(1), obj.loc(1)+obj.dimensions(1);...
                          (2*obj.loc(2)+obj.dimensions(2))/2, (2*obj.loc(2)+obj.dimensions(2))/2];
        end
        function draw(obj)
            rectangle('Position',[obj.loc(1) obj.loc(2) obj.dimensions(1) obj.dimensions(2)])
        end
    end
    
end


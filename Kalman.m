classdef Kalman
% Kalman Filter class
    properties
        xk % state vector
        Ak % 
        Qk %
        Pk % Process
        Kk % Kalman Gain
    end
    methods
        function obj = predict(obj, obs)
        end
        function obj = correct(obj, obs)
        end
    end
end

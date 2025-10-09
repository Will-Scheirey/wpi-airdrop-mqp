classdef Extended_Kalman_Filter < Kalman_Filter
    %EXTENDED_KALMAN_FILTER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        dt
    end

    methods
        function obj = Extended_Kalman_Filter(R, Q, H, x0, P0, dt)
            
            F0 = zeros(numel(x0), numel(x0));
            obj = obj@Kalman_Filter(R, Q, H, F0, x0, P0);

            obj.dt = dt;
            obj.F = obj.f_jacobian_states();
        end            

        function predict(obj)
            obj.F = obj.f_jacobian_states();
            
            obj.x_curr = obj.x_curr + obj.dt * obj.F * obj.x_curr;

            obj.P_curr = obj.F * obj.P_curr * obj.F' + obj.Q;
        end

        function [innovation, K] = update(obj, y)
            y_pred = obj.H * obj.x_curr; % Measurement prediction
            innovation = y - y_pred; % Innovation

            h = obj.h_jacobian_states();

            term1 = obj.R + h*obj.P_curr*h';

            K = obj.P_curr * h' / term1;

            obj.x_curr = obj.x_curr + K * innovation;

            obj.P_curr = (obj.I - K*h) * obj.P_curr * (obj.I - K*h)' + K*obj.R*K'; % Update covariance            
        end

        function step_filter(obj, y)
            obj.predict();
            obj.update(y);
        end
    end

    methods (Abstract)
        dfdx = f_jacobian_states(obj);
        dhdx = h_jacobian_states(obj);
    end
end
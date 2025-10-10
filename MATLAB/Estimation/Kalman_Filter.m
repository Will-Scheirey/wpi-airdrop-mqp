classdef Kalman_Filter < handle
    %KALMANFILTER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        R
        Q

        H
        F

        x_curr

        P_curr

        I
    end

    methods
        function obj = Kalman_Filter(R, Q, H, F, x0, P0)
            obj.R = R;
            obj.Q = Q;
            obj.H = H;
            obj.F = F;

            obj.x_curr = x0;
            obj.P_curr = P0;

            obj.I = eye(size(F));
        end

        function stepFilter(obj, y)
            % Prediction Step
            x_pred = obj.F * obj.x_curr; % State prediction
            P_pred = obj.F * obj.P_curr * obj.F' + obj.Q; % Covariance prediction
            
            % Update Step
            y_pred = obj.H * x_pred; % Measurement prediction
            innovation = y - y_pred; % Innovation
            K = P_pred * obj.H' / (obj.H * P_pred * obj.H' + obj.R); % Kalman Gain
            
            obj.x_curr = x_pred + K * innovation; % Update state estimate
            obj.P_curr = (obj.I - K*obj.H) * P_pred; % Update covariance
        end
    end
end
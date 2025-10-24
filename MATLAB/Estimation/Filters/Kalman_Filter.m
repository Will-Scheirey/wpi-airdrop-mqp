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

        P_hist
        x_hist

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

        function [x_pred, P_pred] = predict(obj)
            x_pred = obj.F * obj.x_curr; % State prediction
            P_pred = obj.F * obj.P_curr * obj.F' + obj.Q; % Covariance prediction
        end

        function [innovation, K] = update(obj, x_pred, y, P_pred)
            % Update Step
            y_pred = obj.H * x_pred; % Measurement prediction
            innovation = y - y_pred; % Innovation
            K = P_pred * obj.H' / (obj.H * P_pred * obj.H' + obj.R); % Kalman Gain
        end

        function update_states(obj, x_pred, K, innovation, P_pred)
            obj.x_curr = x_pred + K * innovation; % Update state estimate
            obj.P_curr = (obj.I - K*obj.H) * P_pred; % Update covariance
        end

        function step_filter(obj, y)
            % Prediction Step
            [x_pred, P_pred] = obj.predict();

            % Update Step
            [innovation, K] = obj.update(x_pred, y, P_pred);

            obj.update_states(x_pred, K, innovation, P_pred);
        end

        function run_filter(obj, y_all, num_steps)

            obj.x_hist = zeros(numel(obj.x_curr), num_steps);
            obj.P_hist = zeros(numel(obj.x_curr), numel(obj.x_curr), num_steps);
            
            for i=1:num_steps
                obj.x_hist(:, i) = obj.x_curr;
                obj.P_hist(:, :, i) = obj.P_curr;
            
                obj.step_filter(y_all(:, i));
            end
            
        end
    end
end
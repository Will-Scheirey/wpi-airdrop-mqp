classdef Extended_Kalman_Filter < handle
    %EXTENDED_KALMAN_FILTER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        dt
        x_inds

        R
        Q

        H
        F

        x_curr

        P_curr

        I

        P_hist
        x_hist
        S_hist

        inno_hist
    end

    methods
        function obj = Extended_Kalman_Filter(R, Q, H, x0, P0, dt, x_inds)

            F0 = zeros(numel(x0), numel(x0));
            
            obj.R = R;
            obj.Q = Q;
            obj.H = H;
            obj.F = F0;

            obj.x_curr = x0;
            obj.P_curr = P0;

            obj.I = eye(size(F0));

            obj.x_inds = x_inds;

            obj.dt = dt;
        end            

        function predict(obj, u)
            A   = obj.f_jacobian_states(u);
            Phi = eye(size(A)) + obj.dt * A;
            % Phi = expm(obj.dt * A);

            Qd = obj.Q;
        
            obj.x_curr = obj.x_curr + obj.dt * obj.f(u);
        
            % Discrete covariance propagation:
            obj.P_curr = Phi * obj.P_curr * Phi' + Qd;
        end

        function [innovation, K, S] = update(obj, y)
            y_pred = obj.h(); % Measurement prediction
            innovation = y - y_pred; % Innovation

            H = obj.h_jacobian_states();

            S = obj.R + H*obj.P_curr*H';

            K = obj.P_curr * H' / S;

            obj.x_curr = obj.x_curr + K * innovation;

            obj.P_curr = (obj.I - K*H) * obj.P_curr * (obj.I - K*H)' + K*obj.R*K'; % Update covariance            
        end

        function [innovation, S] = step_filter(obj, y, u)
            obj.predict(u);
            [innovation, ~, S] = obj.update(y);
        end

        function run_filter(obj, y_all, u_all, num_steps)

            obj.x_hist = zeros(numel(obj.x_curr), num_steps);
            obj.P_hist = zeros(numel(obj.x_curr), numel(obj.x_curr), num_steps);
            obj.inno_hist = zeros(size(y_all));
            obj.S_hist = zeros(height(y_all), height(y_all), num_steps);
            
            for i=1:num_steps
                obj.x_hist(:, i) = obj.x_curr;
            
                [innovation, S] = obj.step_filter(y_all(:, i), u_all(:, i));
                obj.inno_hist(:, i) = innovation;
                obj.P_hist(:, :, i) = obj.P_curr;
                obj.S_hist(:, :, i) = S;
            end
            
        end
    end

    methods (Abstract)
        dfdx = f_jacobian_states(obj, u);
        dhdx = h_jacobian_states(obj);
        y    = h(obj);
        dxdt = f(obj);
    end
end
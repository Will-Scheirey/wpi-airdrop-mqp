classdef Extended_Kalman_Filter < Kalman_Filter
    %EXTENDED_KALMAN_FILTER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        dt
        x_inds
    end

    methods
        function obj = Extended_Kalman_Filter(R, Q, H, x0, P0, dt, x_inds)
            
            F0 = zeros(numel(x0), numel(x0));
            obj = obj@Kalman_Filter(R, Q, H, F0, x0, P0);

            obj.x_inds = x_inds;

            obj.dt = dt;
            obj.F = obj.f_jacobian_states([0; 0; 0]);
        end            

        function predict(obj, u)
            A   = obj.f_jacobian_states(u);
            Phi = eye(size(A)) + obj.dt * A';
            % Phi = expm(obj.dt * A);
        
            % If your Q is continuous-time spectral density, discretize it.
            % Minimal quick fix if Q is already tuned as discrete:
            Qd = obj.Q;
        
            % Nonlinear state propagation (yours):
            obj.x_curr = obj.x_curr + obj.dt * obj.f(u);
        
            % Discrete covariance propagation:
            obj.P_curr = Phi * obj.P_curr * Phi' + Qd;
        end

        function [innovation, K] = update(obj, y)
            y_pred = obj.h(); % Measurement prediction
            innovation = y - y_pred; % Innovation

            H = obj.h_jacobian_states();

            K = obj.P_curr * H' / (obj.R + H*obj.P_curr*H');

            obj.x_curr = obj.x_curr + K * innovation;

            obj.P_curr = (obj.I - K*H) * obj.P_curr * (obj.I - K*H)' + K*obj.R*K'; % Update covariance            
        end

        function step_filter(obj, y, u)
            obj.predict(u);
            obj.update(y);
        end

        function run_filter(obj, y_all, u_all, num_steps)

            obj.x_hist = zeros(numel(obj.x_curr), num_steps);
            obj.P_hist = zeros(numel(obj.x_curr), numel(obj.x_curr), num_steps);
            
            for i=1:num_steps
                obj.x_hist(:, i) = obj.x_curr;
                obj.P_hist(:, :, i) = obj.P_curr;
            
                obj.step_filter(y_all(:, i), u_all(:, i));
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
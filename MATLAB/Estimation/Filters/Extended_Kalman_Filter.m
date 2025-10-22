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
            A   = obj.f_jacobian_states();
            Phi = eye(size(A)) + obj.dt * A';
            % Phi = expm(obj.dt * A);
        
            % If your Q is continuous-time spectral density, discretize it.
            % Minimal quick fix if Q is already tuned as discrete:
            Qd = obj.Q;
        
            % Nonlinear state propagation (yours):
            obj.x_curr = obj.x_curr + obj.dt * obj.f();
        
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

        function step_filter(obj, y)
            obj.predict();
            obj.update(y);
        end
    end

    methods (Abstract)
        dfdx = f_jacobian_states(obj);
        dhdx = h_jacobian_states(obj);
        y    = h(obj);
        dxdt = f(obj);
    end
end
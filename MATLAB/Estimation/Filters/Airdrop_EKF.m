classdef Airdrop_EKF < Airdrop_Filter
    % AIRDROP_EKF Implemntation of the Airdrop_Filter class as an EKF
    %   This class implements the basic EKF equations using a first-order
    %   Taylor Series expansion of the matrix exponential. Higher-order
    %   predictions may improve performance

    methods

        function obj = Airdrop_EKF(R, Q, H0, P0, dt)
            obj = obj@Airdrop_Filter(R, Q, H0, P0, dt);
        end

        function [innovation, K, S] = update_impl(obj, y, y_pred, H, R_meas)
            % UPDATE_IMPL Implements the basic EKF update equations
            %
            % INPUTS:
            %   obj    : The EKF object
            %   y      : The measurement
            %   y_pred : The predicted measurement
            %   H      : The measurement Jacobian
            %   R_mesa : The measurement covariance matrix
            % 
            % OUTPUTS:
            %   innovation : The innovation (y - y_pred)
            %   K          : The calculated Kalman gain matrix
            %   S          : The innovation covariance matrix


            % Calculate innovation covariance and Kalman gains
            S = R_meas + H*obj.P_curr*H';
            K = obj.P_curr * H' / S;

            % Calculate innovation and run update step
            innovation = y - y_pred;
            obj.x_curr = obj.x_curr + K * innovation;
            obj.P_curr = (obj.I - K*H) * obj.P_curr * (obj.I - K*H)' + K*R_meas*K';
        end

        function predict_impl(obj, u)
            % PREDICT_IMPL Implements the basic EKF prediction equations
            %
            % INPUTS:
            %   obj : The EKF object
            %   u   : The inputs for the current step
            %

            % Calculate the jacobian and the state transition matrix 
            A   = obj.f_jacobian_states(u);
            Phi = eye(size(A)) + obj.dt * A;

            % For now just assume constant Qd
            Qd = obj.Q;

            % Run the state and covariance prediction step 
            obj.x_curr = obj.x_curr + obj.dt * obj.f(u);
            obj.P_curr = Phi * obj.P_curr * Phi' + Qd;
        end
    end
end
classdef Airdrop_EKF < Airdrop_Filter

    properties
        F_hist
        Q_hist
    end

    methods

        function obj = Airdrop_EKF(R, Q, H0, P0, dt)
            obj = obj@Airdrop_Filter(R, Q, H0, P0, dt);
        end

        function [innovation, K, S] = update_impl(obj, y, y_pred, H, R_meas)
            innovation = y - y_pred;

            S = R_meas + H*obj.P_curr*H';

            K = obj.P_curr * H' / S;

            obj.x_curr = obj.x_curr + K * innovation;

            obj.P_curr = (obj.I - K*H) * obj.P_curr * (obj.I - K*H)' + K*R_meas*K';
        end

        function predict_impl(obj, u)
            A   = obj.f_jacobian_states(u);
            Phi = eye(size(A)) + obj.dt * A;
            % Phi = expm(obj.dt * A);

            Qd = obj.Q;

            k = obj.hist_idx;
            if isempty(obj.F_hist)
                nx = size(Phi,1);

                obj.F_hist = NaN(nx,nx,100000); % replace with real N allocation
                obj.Q_hist = NaN(nx,nx,100000);
            end
            obj.F_hist(:,:,k) = Phi;
            obj.Q_hist(:,:,k) = Qd;

            obj.x_curr = obj.x_curr + obj.dt * obj.f(u);

            % Discrete covariance propagation:
            obj.P_curr = Phi * obj.P_curr * Phi' + Qd;
        end
    end
end
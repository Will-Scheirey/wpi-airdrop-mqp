classdef Forward_Backward_Smoother < Airdrop_Filter
    % Backward Information Filter + fusion smoother (two-filter smoother)
    % Uses forward EKF histories (x_f^+, P_f^+, F_k, Q_k) and runs a backward
    % information pass that accumulates "future" information, then fuses:
    %
    %   P_s = inv(inv(P_f^+) + I_b^-)
    %   x_s = P_s * (inv(P_f^+)*x_f^+ + s_b^-)
    %
    % Key fixes vs your current version:
    %   (1) info measurement update uses y_lin (NOT raw y)
    %   (2) backward time update uses x_k = inv(I_k^+) * s_k^+
    %       and term = (Q + inv(I_k^+))^{-1} via the Qi - Qi*(I+Qi)^{-1}*Qi identity
    %   (3) avoids inv() and fixes the algebra ordering

    properties
        % Provided from forward EKF
        x_f_plus        % nx x N
        P_f_plus        % nx x nx x N
        F_hist          % nx x nx x (N-1)   (Phi_{k-1}: (k-1)->k)
        Q_hist          % nx x nx x (N-1)

        % Backward info state (at each time index k)
        I_b_minus       % nx x nx x N   (I_{b,k}^-)
        s_b_minus       % nx x N        (s_{b,k}^-)

        % Per-step working accumulators (I_{b,k}^+, s_{b,k}^+)
        I_b_plus_curr
        s_b_plus_curr
        did_time_update_this_step

        % Output
        x_smooth
        P_smooth

        step_i

        % numerics
        rcond_tol = 1e-12;
    end

    methods
        function obj = Forward_Backward_Smoother(R, Q, H0, P0, dt)
            obj = obj@Airdrop_Filter(R, Q, H0, P0, dt);
        end

        function set_forward_results(obj, x_f_plus, P_f_plus, F_hist, Q_hist)
            obj.x_f_plus = x_f_plus;
            obj.P_f_plus = P_f_plus;
            obj.F_hist   = F_hist;
            obj.Q_hist   = Q_hist;
        end

        function smooth(obj, y_all, u_all, timesteps, acc_gps_all, verbose)
            if nargin < 6, verbose = false; end
            if isempty(obj.x_f_plus) || isempty(obj.P_f_plus) || isempty(obj.F_hist) || isempty(obj.Q_hist)
                error("Call set_forward_results(...) with forward EKF histories first.");
            end

            nx = size(obj.x_f_plus, 1);
            N  = size(obj.x_f_plus, 2);

            % Backward init: I_{b,N}^- = 0, s_{b,N}^- = 0 (no future info beyond end)
            obj.I_b_minus = zeros(nx, nx, N);
            obj.s_b_minus = zeros(nx, N);

            % Linearize "at" the forward state
            obj.x_curr = obj.x_f_plus(:, N);

            % Run backward with update-then-predict so measurements at k are included in I_k^+
            obj.run_filter(y_all, u_all, timesteps, acc_gps_all, 0, -1, verbose, "update_then_predict");
        end

        function [x_sm, P_sm] = fuse(obj)
            obj.fuse_all_times();
            x_sm = obj.x_smooth;
            P_sm = obj.P_smooth;
        end
    end

    % ------------------------------------------------------------
    % Hooks for Airdrop_Filter iterator
    % ------------------------------------------------------------
    methods (Access = protected)
        function pre_step(obj, i)
            obj.step_i = i;

            % Keep the backward pass linearized around the forward estimate at i
            obj.x_curr = obj.x_f_plus(:, i);

            % Start this step's info accumulators from stored I^- and s^-
            obj.I_b_plus_curr = obj.I_b_minus(:, :, i);
            obj.s_b_plus_curr = obj.s_b_minus(:, i);

            obj.did_time_update_this_step = false;
        end

        function post_step(obj, i) %#ok<INUSD>
            % no-op
        end
    end

    % ------------------------------------------------------------
    % Information-form measurement update + backward time update
    % ------------------------------------------------------------
    methods
        function [innovation, K, S] = update_impl(obj, y, y_pred, H, R_meas)
            % Backward measurement update in information form:
            %   I^+ = I^- + H' R^{-1} H
            %   s^+ = s^- + H' R^{-1} y_lin
            %
            % For nonlinear h(x): y_lin = y - h(xbar) + H*xbar
            xbar = obj.x_curr;

            y_lin = y - y_pred + H*xbar;

            % I += H' * R^{-1} * H
            obj.I_b_plus_curr = obj.I_b_plus_curr + H'*(R_meas\H);

            % s += H' * R^{-1} * y_lin   (FIX: use y_lin, not raw y)
            obj.s_b_plus_curr = obj.s_b_plus_curr + H'*(R_meas\y_lin);

            % outputs only for logging/compatibility
            innovation = y - y_pred;
            K = [];
            S = R_meas;
        end

        function predict_impl(obj, u) %#ok<INUSD>
            % Backward time update (propagate info from k -> k-1).
            %
            % Let forward dynamics be: x_k = F_{k-1} x_{k-1} + w,  w~N(0,Q_{k-1})
            %
            % Given future info at time k: (I_k^+, s_k^+), with
            %   Pk = inv(I_k^+),  xk = Pk*s_k^+
            %
            % Induced likelihood on x_{k-1} is:
            %   xk ~ N(F x_{k-1}, Q + Pk)
            % so
            %   I_{k-1}^- = F' * (Q + Pk)^{-1} * F
            %   s_{k-1}^- = F' * (Q + Pk)^{-1} * xk
            %
            % We compute (Q + Pk)^{-1} without forming Pk explicitly:
            %   (Q + inv(I))^{-1} = Qi - Qi*(I + Qi)^{-1}*Qi, where Qi = inv(Q)
            %
            if obj.did_time_update_this_step
                return;
            end
            obj.did_time_update_this_step = true;

            k = obj.step_i;
            if isempty(k) || k <= 1
                return;
            end

            F = obj.F_hist(:, :, k-1);   % (k-1)->k
            Q = obj.Q_hist(:, :, k-1);

            Iplus = obj.I_b_plus_curr;
            splus = obj.s_b_plus_curr;

            nx = size(F,1);
            I  = eye(nx);

            % If we have no usable info yet, don't propagate anything backward
            if norm(Iplus, 'fro') == 0 || ~isfinite(rcond(Iplus)) || rcond(Iplus) < obj.rcond_tol
                obj.I_b_minus(:, :, k-1) = zeros(nx);
                obj.s_b_minus(:, k-1)    = zeros(nx,1);
                return;
            end

            % xk = inv(Iplus) * splus  (solve, not inv)
            xk = Iplus \ splus;

            % Qi = inv(Q) (solve, not inv)
            Qi = Q \ I;

            % term = (Q + inv(Iplus))^{-1} = Qi - Qi*(Iplus + Qi)^{-1}*Qi
            M    = Iplus + Qi;
            term = Qi - Qi * (M \ Qi);

            % I_{k-1}^- and s_{k-1}^-
            Iminus_prev = F' * term * F;
            sminus_prev = F' * term * xk;

            obj.I_b_minus(:, :, k-1) = Iminus_prev;
            obj.s_b_minus(:, k-1)    = sminus_prev;
        end
    end

    % ------------------------------------------------------------
    % Fusion
    % ------------------------------------------------------------
    methods (Access = private)
        function fuse_all_times(obj)
            nx = size(obj.x_f_plus, 1);
            N  = size(obj.x_f_plus, 2);

            obj.x_smooth = NaN(nx, N);
            obj.P_smooth = NaN(nx, nx, N);

            I = eye(nx);

            for k = 1:N
                Pf = obj.P_f_plus(:,:,k);
                xf = obj.x_f_plus(:,k);

                Ib = obj.I_b_minus(:,:,k);
                sb = obj.s_b_minus(:,k);

                if norm(Ib, 'fro') == 0
                    obj.x_smooth(:,k)   = xf;
                    obj.P_smooth(:,:,k) = Pf;
                    continue;
                end

                % P = inv(inv(Pf) + Ib)
                invPf = Pf \ I;
                P = (invPf + Ib) \ I;

                % x = P * (inv(Pf)*xf + sb)
                x = P * (invPf*xf + sb);

                obj.x_smooth(:,k)   = x;
                obj.P_smooth(:,:,k) = P;
            end
        end
    end
end
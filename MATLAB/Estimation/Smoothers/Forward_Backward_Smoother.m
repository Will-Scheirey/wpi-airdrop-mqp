classdef Forward_Backward_Smoother < Airdrop_Filter
    % Backward Information Filter + fusion smoother (two-filter smoother)
    % Implements the algorithm in your screenshot (Eqs 9.75–9.78 style).

    properties
        % Provided from forward EKF
        x_f_plus        % nx x N
        P_f_plus        % nx x nx x N
        F_hist          % nx x nx x (N-1)   (Phi_k mapping k -> k+1 in forward)
        Q_hist          % nx x nx x (N-1)

        % Backward info state
        I_b_minus       % nx x nx x N   (T_{b,k}^-)
        s_b_minus       % nx x N        (s_k^-)

        % Per-step working accumulators (T_{b,k}^+, s_k^+)
        I_b_plus_curr
        s_b_plus_curr
        did_time_update_this_step

        % Output
        x_smooth
        P_smooth

        step_i
    end

    methods
        function obj = Forward_Backward_Smoother(R, Q, H0, P0, dt, J)
            obj = obj@Airdrop_Filter(R, Q, H0, P0, dt, J);
        end

        function set_forward_results(obj, x_f_plus, P_f_plus, F_hist, Q_hist)
            obj.x_f_plus = x_f_plus;
            obj.P_f_plus = P_f_plus;
            obj.F_hist   = F_hist;
            obj.Q_hist   = Q_hist;
        end

        function smooth(obj, y_all, u_all, timesteps, acc_gps_all, verbose)
            if nargin < 6, verbose = false; end
            if isempty(obj.x_f_plus) || isempty(obj.P_f_plus)
                error("Call set_forward_results(...) with forward EKF histories first.");
            end

            nx = size(obj.x_f_plus, 1);
            N  = size(obj.x_f_plus, 2);

            % Backward init (Eq 9.75): s_N^- = 0, I_bN^- = 0
            obj.I_b_minus = zeros(nx, nx, N);
            obj.s_b_minus = zeros(nx, N);

            % Start at time N (for linearization)
            obj.x_curr = obj.x_f_plus(:, N);

            % Run the superclass iterator backward, but with update-before-predict order
            obj.run_filter(y_all, u_all, timesteps, acc_gps_all, -1, verbose, "update_then_predict");
        end
        function [x_sm, P_sm] = fuse(obj)
           % Fuse forward + backward (Eq 9.78)
            % obj.fuse_all_times1(1e-12);
            obj.fuse_all_times();

            x_sm = obj.x_smooth;
            P_sm = obj.P_smooth;
        end
    end

    % ------------------------------------------------------------
    % Hook: align linearization/state for step i
    % ------------------------------------------------------------
    methods (Access = protected)
        function pre_step(obj, i)
            obj.step_i = i;

            obj.x_curr = obj.x_f_plus(:, i);

            obj.I_b_plus_curr = obj.I_b_minus(:, :, i);
            obj.s_b_plus_curr = obj.s_b_minus(:, i);

            obj.did_time_update_this_step = false;
        end

        function post_step(obj, i) %#ok<INUSD>
            % no-op
        end
    end

    % ------------------------------------------------------------
    % Required Airdrop_Filter implementations: now info-form math
    % ------------------------------------------------------------
    methods
        function [innovation, K, S] = update_impl(obj, y, y_pred, H, R_meas)
            % Implements:
            % I^+ = I^- + H' R^-1 H
            % s^+ = s^- + H' R^-1 y_lin
            %
            % For nonlinear h(x), use y_lin = y - h(xbar) + H*xbar
            xbar = obj.x_curr;

            y_lin = y - y_pred + H * xbar;

            obj.I_b_plus_curr = obj.I_b_plus_curr + H' * (R_meas \ H);
            obj.s_b_plus_curr = obj.s_b_plus_curr + H' * (R_meas \ y_lin);

            % Return values just for logging/compatibility
            innovation = y - y_pred;
            K = [];
            S = R_meas;
        end

        function predict_impl(obj, u) %#ok<INUSD>
            % Backward time update (Eq 9.76):
            % I_{k-1}^- = F_{k-1}' [Q^{-1} - Q^{-1}(I_k^+ + Q^{-1})^{-1}Q^{-1}] F_{k-1}
            % s_{k-1}^- = I_{k-1}^- F_{k-1}^{-1} (I_k^+)^{-1} s_k^+

            if obj.did_time_update_this_step
                return; % ensure one time-update per step/bin
            end
            obj.did_time_update_this_step = true;

            k = obj.step_i;      % TRUE timestep index from run_filter via pre_step
            if isempty(k) || k <= 1
                return;          % no k-1 to write
            end

            F = obj.F_hist(:, :, k-1);   % forward transition from (k-1) -> k
            Q = obj.Q_hist(:, :, k-1);   % process cov for that transition

            Iplus = obj.I_b_plus_curr;   % I_{b,k}^+
            splus = obj.s_b_plus_curr;   % s_k^+

            nx = size(F,1);
            I = eye(nx);

            % Compute Q^{-1} via solve
            Qi = Q \ I;                  % Qi = inv(Q)

            % term = Qi - Qi*(Iplus + Qi)^{-1}*Qi
            M = Iplus + Qi;
            term = Qi - Qi * (M \ Qi);

            Iminus_prev = F' * term * F; % I_{b,k-1}^-

            % s_{k-1}^- = I_{k-1}^- * F^{-1} * (I_k^+)^{-1} * s_k^+
            % Solve v = (Iplus)^(-1) * splus, but only if well-conditioned
            if rcond(Iplus) < 1e-12
                % No usable information yet
                v = zeros(size(splus));
            else
                v = Iplus \ splus;
            end

            w = F \ v;                   % w = F^{-1} v
            sminus_prev = Iminus_prev * w;

            obj.I_b_minus(:, :, k-1) = Iminus_prev;
            obj.s_b_minus(:, k-1)    = sminus_prev;
        end
    end

    % ------------------------------------------------------------
    % Private utilities
    % ------------------------------------------------------------
    methods (Access = private)
        function fuse_all_times(obj)
            nx = size(obj.x_f_plus, 1);
            N  = size(obj.x_f_plus, 2);

            obj.x_smooth = NaN(nx, N);
            obj.P_smooth = NaN(nx, nx, N);

            I = eye(nx);

            for k = 1:N
                fprintf("Fusing: %d/%d\n", k, N)
                Pf = obj.P_f_plus(:,:,k);
                xf = obj.x_f_plus(:,k);

                Ib = obj.I_b_minus(:,:,k);
                sb = obj.s_b_minus(:,k);

                if norm(Ib, 'fro') == 0
                    obj.x_smooth(:,k) = xf;
                    obj.P_smooth(:,:,k) = Pf;
                    continue;
                end

                % Backward covariance is Pb = inv(Ib)
                % Fusion:
                % P = inv(inv(Pf) + Ib)
                invPf = Pf \ I;
                P = (invPf + Ib) \ I;

                x = P * (invPf*xf + sb);

                obj.x_smooth(:,k) = x;
                obj.P_smooth(:,:,k) = P;
            end
        end

        function fuse_all_times1(obj, rcond_tol)
            %FUSE_ALL_TIMES Fuse forward EKF (x_f^+, P_f^+) with backward info (I_b^-, s_b^-)
            % using the exact equations from your screenshot (Eq 9.78):
            %
            %   P_bm^- = (I_bm^-)^{-1}
            %   x_bm^- = P_bm^- s_bm^-
            %   K_f    = P_bm^- (P_fm^+ + P_bm^-)^{-1}
            %   x_m    = K_f x_fm^+ + (I - K_f) x_bm^-
            %   P_m    = [ (P_fm^+)^{-1} + (P_bm^-)^{-1} ]^{-1}
            %
            % Results are stored in:
            %   obj.x_smooth, obj.P_smooth
            % Optionally also stores:
            %   obj.K_f_hist, obj.P_bm_hist, obj.x_bm_hist  (if those properties exist)
            %
            % If I_bm^- is singular/ill-conditioned, falls back to forward:
            %   x_m = x_fm^+, P_m = P_fm^+, K_f = 0.

            if nargin < 2 || isempty(rcond_tol)
                rcond_tol = 1e-12;
            end

            nx = size(obj.x_f_plus, 1);
            N  = size(obj.x_f_plus, 2);

            I = eye(nx);

            obj.x_smooth = NaN(nx, N);
            obj.P_smooth = NaN(nx, nx, N);

            % Optional histories if you added these properties
            hasK  = isprop(obj, "K_f_hist");
            hasPb = isprop(obj, "P_bm_hist");
            hasXb = isprop(obj, "x_bm_hist");

            if hasK,  obj.K_f_hist  = zeros(nx, nx, N); end
            if hasPb, obj.P_bm_hist = NaN(nx, nx, N);  end
            if hasXb, obj.x_bm_hist = NaN(nx, N);      end

            for m = 1:N
                xf = obj.x_f_plus(:, m);
                Pf = obj.P_f_plus(:, :, m);

                Ib = obj.I_b_minus(:, :, m);   % (P_bm^-)^{-1}
                sb = obj.s_b_minus(:, m);      % s_bm^-

                % If there is no backward information, smoothed = forward
                if norm(Ib, 'fro') == 0 || ~isfinite(rcond(Ib)) || rcond(Ib) < rcond_tol
                    obj.x_smooth(:, m)     = xf;
                    obj.P_smooth(:, :, m)  = Pf;
                    if hasK, obj.K_f_hist(:, :, m) = zeros(nx); end
                    continue;
                end

                % --- Backward covariance and mean ---
                % P_bm^- = inv(Ib), x_bm^- = P_bm^- * sb
                Pb = Ib \ I;
                xb = Pb * sb;

                if hasPb, obj.P_bm_hist(:, :, m) = Pb; end
                if hasXb, obj.x_bm_hist(:, m)    = xb; end

                % --- Eq (9.78) covariance ---
                invPf = Pf \ I;
                Pm = (invPf + Ib) \ I;

                % --- Eq (9.78) fusion gain ---
                % K_f = P_bm^- (P_fm^+ + P_bm^-)^{-1}
                Kf = Pb / (Pf + Pb);

                % --- Eq (9.78) fused state ---
                xm = Kf * xf + (I - Kf) * xb;

                obj.x_smooth(:, m)    = xm;
                obj.P_smooth(:, :, m) = Pm;

                if hasK, obj.K_f_hist(:, :, m) = Kf; end
            end
        end
    end
end
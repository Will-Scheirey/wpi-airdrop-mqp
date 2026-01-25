classdef Airdrop_Filter < Abstract_Filter

    properties
        % --- Constants ---
        J
        g_norm      = 9.80665;
        accel_gate  = 1e0;
        alt_gate    = 1e10;
        speed_gate  = 3
        g_vec_e
        dt
        I

        % --- Measurement Matrices ---
        measurement_ranges

        dhdx_p
        dhdx_mag
        dhdx_w
        dhdx_alt
        dhdx_w_no_bias
        dhdx_pos_no_bias

        % --- Filter Properties

        meas_defs
        is_initialized
        num_states

        x_curr
        P_curr

        H
        F

        update_a_b
        last_down
        last_u

        m_ref_i  % 3x1 reference mag in inertial

        % --- Histories ---
        hist_idx
        accel_calc_all
        trust_accel_all
        quat_meas_all
        down_vec_all

        P_hist
        x_hist
        S_hist

        inno_hist
    end

    methods (Abstract)
        update_impl(y, y_pred, H, R)
        predict_impl(u)
    end

    methods (Access = protected)
        function pre_step(obj, i)
            % Hook for subclasses. Default: no-op.
        end

        function post_step(obj, i)
            % Hook for subclasses. Default: no-op.
        end
    end

    methods

        function obj = Airdrop_Filter(R, Q, H0, P0, dt, J)
            obj.J = J;
            obj.R = R;
            obj.Q = Q;
            obj.H = H0;

            obj.P_curr = P0;

            obj.I = eye(size(P0));

            obj.dt = dt;

            obj.g_vec_e = [0; 0; -obj.g_norm];

            obj.last_u = struct('accel', [0, 0, 0], 'gyro', [0, 0, 0]);
            obj.last_down = [0; 0; 1];
            obj.update_a_b = true;

            obj.init_x_inds();
            obj.init_y_inds();

            obj.is_initialized = false;

            obj.hist_idx = 1;
        end

        % --- SETUP ---

        function init_x_inds(obj)

            state_blocks = {
                "P_E", 3; ... % Position in Inertial Frame
                "V_E", 3; ... % Velocity in Inertial Frame
                "e",   4; ... % Quaternion
                 ... % "w_b", 3; ... % Angular Velocity
                "b_g", 3; ... % Gyro Bias
                "b_a", 3; ... % Accelerometer Bias
                "b_p", 3; ... % GPS Position Bias
                "b_m", 3; ... % Magnetometer Bias
                };

            state_idx = 1;
            for i=1:size(state_blocks,1)
                state_name = state_blocks{i,1};
                state_size  = state_blocks{i,2};
                obj.x_inds.(state_name) = (state_idx:(state_idx+state_size-1))';
                state_idx = state_idx + state_size;
            end

            obj.num_states = state_idx - 1;
        end

        function init_y_inds(obj)
            obj.meas_defs = struct( ...
                "pos", struct("idx",1,"dim",3), ...
                "mag", struct("idx",2,"dim",3), ...
                ... % "gyro",struct("idx",3,"dim",3), ...
                "alt", struct("idx",3,"dim",1));

            r = 1;
            for k = 1:numel(fieldnames(obj.meas_defs))
                fn = fieldnames(obj.meas_defs);
                s = obj.meas_defs.(fn{k});
                obj.measurement_ranges{s.idx} = r:(r+s.dim-1);
                r = r + s.dim;
            end
        end

        function initialize(obj, stationary, accel_meas, gyro_meas, mag_meas, gps_meas, baro_meas)
            pos_inds = obj.x_inds.P_E;

            % Initialize position to the GPS measurement
            obj.x_curr(pos_inds(1:2)) = gps_meas(1:2);

            % Initialize altitude to the baro measurement
            obj.x_curr(pos_inds(3))   = baro_meas;

            % Initialize altitude bias to GPS altitude minus baro altitude
            obj.x_curr(obj.x_inds.b_p(3)) = gps_meas(3) - baro_meas;

            if ~stationary, return; end

            % Rescale accel to have the expected norm
            accel_meas_corr = obj.g_norm * accel_meas / norm(accel_meas);

            % Change alt gate so we can calculate the down vector
            old_alt_gate = obj.alt_gate;
            obj.alt_gate = 1e10;

            % Estimate the orientation from accel and mag measurments
            q_meas = obj.quat_from_acc_mag(accel_meas_corr, mag_meas);

            % C_i_b = ecef2body_rotm(q_meas);   % IMPORTANT: check your function name returns which direction
            % You used C_BE in your quat_from_acc_mag and called it C_BE.
            % In your calc_accel you do: C_EB = ecef2body_rotm(e)' ; then a_e = C_EB*(a-b) + g
            % That implies ecef2body_rotm(e) returns C_BE (body <- earth). Good.

            C_bi = ecef2body_rotm(q_meas);     % body -> inertial (b2i)
            m_b0 = mag_meas / norm(mag_meas);  % measured mag in body

            % m_b = C_ib * m_i  =>  m_i = C_bi * m_b
            obj.m_ref_i = C_bi * m_b0;         % inertial reference mag

            % debug_mag_convention_once(accel_meas_corr, mag_meas, obj.m_ref_i);

            % Change alt gate back
            obj.alt_gate = old_alt_gate;

            % Initialize velocity and angular velocity to zero
            obj.x_curr(obj.x_inds.V_E) = zeros(3,1);
            % obj.x_curr(obj.x_inds.w_b) = zeros(3,1);

            % Initialize gyro bias to the current gyro readings
            obj.x_curr(obj.x_inds.b_g) = gyro_meas;
            % Initialize orientation
            obj.x_curr(obj.x_inds.e) = q_meas;
            % Initialize accel bias to the measurement minus corrected
            obj.x_curr(obj.x_inds.b_a) = accel_meas - accel_meas_corr;
            obj.x_curr(obj.x_inds.b_m) = zeros(3,1);

            obj.x_curr = obj.x_curr(:);

            obj.is_initialized = true;
        end

        % --- GETTERS ---

        function P_E_out = get_P_E(obj)
            P_E_out = obj.x_curr(obj.x_inds.P_E);
        end

        function V_E_out = get_V_E(obj)
            V_E_out = obj.x_curr(obj.x_inds.V_E);
        end

        function e_out = get_e(obj)
            e_out   = obj.x_curr(obj.x_inds.e);
        end

        function w_b_out = get_w_b(obj)
            w_b_out  = obj.x_curr(obj.x_inds.w_b);
        end

        function b_g_out = get_b_g(obj)
            b_g_out = obj.x_curr(obj.x_inds.b_g);
        end

        function b_a_out = get_b_a(obj)
            b_a_out = obj.x_curr(obj.x_inds.b_a);
        end

        function b_p_out = get_b_p(obj)
            b_p_out = obj.x_curr(obj.x_inds.b_p);
        end

        function b_m_out = get_b_m(obj)
            b_m_out = obj.x_curr(obj.x_inds.b_m);
        end

        function alt_out = altitude(obj)
            pos = obj.get_P_E();
            alt_out = pos(3);
        end

        function speed_out = speed(obj)
            speed_out = norm(obj.get_V_E);
        end

        function run_filter(obj, y_all, u_all, timesteps, acc_gps_all, drop_time, direction, verbose, order)
            %RUN_FILTER Run filter forward/backward with multiple async measurement streams.
            %
            % direction: +1 for forward, -1 for backward
            % order: "predict_then_update" (default) or "update_then_predict"
            %
            % Adds subclass hooks:
            %   obj.pre_step(i)
            %   obj.post_step(i)

            if ~obj.is_initialized
                error("Initialize the filter before running!!");
            end

            if nargin < 7 || isempty(direction), direction = +1; end
            if nargin < 8, verbose = false; end
            if nargin < 9 || isempty(order), order = "predict_then_update"; end

            direction = sign(direction);
            if direction == 0, direction = +1; end

            num_steps = numel(timesteps);
            if num_steps < 2
                error("timesteps must have at least 2 elements.");
            end

            % Base dt magnitude from adjacent samples (assumes uniform)
            dt_mag = abs(timesteps(2) - timesteps(1));
            dt_ = direction * dt_mag;

            % Temporarily set obj.dt so predict() uses correct sign
            dt_old = obj.dt;
            obj.dt = dt_;

            nx = numel(obj.x_curr);

            % --- Pre-allocate histories ---
            obj.x_hist = zeros(nx, num_steps);
            obj.P_hist = zeros(nx, nx, num_steps);

            inno_height = 0;
            for k = 1:numel(y_all)
                inno_height = inno_height + width(y_all{k}.data);
            end
            obj.inno_hist = NaN(inno_height, num_steps);

            % S per update (variable count), store as cell
            obj.S_hist = cell(num_steps, numel(y_all));

            obj.accel_calc_all  = zeros(num_steps, 3);
            obj.trust_accel_all = NaN(num_steps, 1);
            obj.down_vec_all    = NaN(num_steps, 3);
            obj.quat_meas_all   = NaN(num_steps, 4);

            % Indices of timesteps in run order
            if direction > 0
                step_inds = 1:num_steps;
            else
                step_inds = num_steps:-1:1;
            end

            % Pointers for each stream (next unread row depends on direction)
            y_ptr = zeros(numel(y_all), 1);
            for n = 1:numel(y_all)
                if direction > 0
                    y_ptr(n) = 1;                    % next unread (forward)
                else
                    y_ptr(n) = height(y_all{n});     % next unread (backward)
                end
            end
            if direction > 0
                u_ptr = 1;
            else
                u_ptr = height(u_all);
            end

            obj.hist_idx = 1;

            % --- Main loop ---
            for ii = 1:numel(step_inds)
                i = step_inds(ii);

                % ---- subclass hook ----
                obj.pre_step(i);

                t0 = timesteps(i);
                t1 = t0 + dt_;  % forward: t1>t0, backward: t1<t0

                
                if t0 > drop_time
                    % obj.R(4:7, 4:7) = (eye(4,4) * 1e-6) .^2;
                    % obj.R(11, 11)   = 300;
                    obj.Q(obj.x_inds.b_m, obj.x_inds.b_m) = 1e-5 * eye(3);
                end

                % Define bin as [tmin, tmax)
                tmin = min(t0, t1);
                tmax = max(t0, t1);

                if verbose && mod(ii, 1) == 0
                    fprintf("%d / %d (t=%.6f)\n", ii, num_steps, t0);
                end

                % Log state before propagation (matches your original style)
                obj.x_hist(:, i) = obj.x_curr;

                % Helper lambdas to keep code readable
                do_predict_block = @() predict_block();
                do_update_block  = @() update_block(i);

                % Execute in requested order
                if order == "predict_then_update"
                    do_predict_block();
                    do_update_block();
                elseif order == "update_then_predict"
                    do_update_block();
                    do_predict_block();
                else
                    error("Unknown order '%s'. Use 'predict_then_update' or 'update_then_predict'.", order);
                end

                obj.P_hist(:, :, i) = obj.P_curr;

                % ---- subclass hook ----
                obj.post_step(i);

                obj.hist_idx = obj.hist_idx + 1;
            end

            % Restore obj.dt
            obj.dt = dt_old;

            % ===================== nested helper functions =====================

            function predict_block()
                % 1) Apply all control inputs whose timestamps fall in [tmin, tmax)
                if ~isempty(u_all)
                    [u_ptr, u_rows] = obj.take_rows_in_window_dir(u_all.time, u_ptr, tmin, tmax, direction);

                    % Apply in correct temporal order for the pass
                    for kk = 1:numel(u_rows)
                        u = u_all(u_rows(kk), :);
                        obj.predict(u);
                    end

                    if isempty(u_rows)
                        obj.predict([]);
                    end
                else
                    obj.predict([]);
                end
            end

            function update_block(step_i)
                % 2) Apply all measurements (each stream) in [tmin, tmax)
                for n = 1:numel(y_all)
                    yn = y_all{n};
                    if isempty(yn)
                        continue;
                    end

                    [y_ptr(n), meas_rows] = obj.take_rows_in_window_dir(yn.time, y_ptr(n), tmin, tmax, direction);
                    if isempty(meas_rows)
                        continue;
                    end

                    S_list = cell(numel(meas_rows), 1);

                    for kk = 1:numel(meas_rows)
                        r = meas_rows(kk);
                        meas = yn(r, :);
                        if isempty(meas)
                            continue;
                        end

                        meas_idx = meas.meas_idx;

                        if meas_idx == 1
                            % Assumes acc_gps_all is aligned with this stream's row index.
                            accH = acc_gps_all(r, 1);
                            accV = acc_gps_all(r, 2);
                            R_gps = (blkdiag(accH, accH, accV)).^2;
                            % Rgps = 20;
                            % R_gps = eye(3) * Rgps^2;

                            [innovation, ~, S] = obj.update(meas.data', meas_idx, R_gps);
                        else
                            [innovation, ~, S] = obj.update(meas.data', meas_idx);
                        end

                        obj.inno_hist(obj.measurement_ranges{meas_idx}, obj.hist_idx) = innovation;
                        S_list{kk} = S;
                    end

                    obj.S_hist{step_i, n} = S_list;
                end
            end
        end

        % --- FILTER MATH ---

        function [innovation, K, S] = update(obj, y, meas_idx, R_gps)
            y_pred = obj.h(meas_idx);
            H = obj.h_jacobian_states(meas_idx);

            range = obj.measurement_ranges{meas_idx};
            R_meas = obj.R(range, range);
            if nargin == 4
                R_meas = R_gps;
            end

            [innovation, K, S] = obj.update_impl(y, y_pred, H, R_meas);
            obj.normalize_quat();
        end
        function predict(obj, u)
            if isempty(u)
                u = obj.last_u;
            else
                obj.last_u = u;
            end

            obj.predict_impl(u);
            obj.normalize_quat();
        end

        function A = f_jacobian_states(obj, u)
            %F_JACOBIAN_STATES  Continuous-time state Jacobian (df/dx)
            % Rewritten to be maintainable, but it uses the EXACT SAME equations
            % as your existing dx0dx..dx21dx construction.
            %
            % Works for both your old 22-state layout and your newer layouts (e.g. if you
            % added b_m). Any extra states beyond the original ones get zero dynamics here.

            % --- size / indices ---
            nx = numel(obj.x_curr);
            A  = zeros(nx, nx);

            % Required groups (must exist in your init_x_inds)
            P  = obj.x_inds.P_E(:);   % 3
            V  = obj.x_inds.V_E(:);   % 3
            E  = obj.x_inds.e(:);     % 4  (e0 e1 e2 e3)
            % W  = obj.x_inds.w_b(:);   % 3  (w0 w1 w2)
            BG = obj.x_inds.b_g(:);   % 3
            BA = obj.x_inds.b_a(:);   % 3
            BP = obj.x_inds.b_p(:);   % 3

            % ---- your gating logic for accel-bias coupling (same as current code) ----
            a_b_mult = true;
            if obj.altitude() > obj.alt_gate || obj.speed() > obj.speed_gate
                a_b_mult = false;
            end

            % --- pull current state (same variable names) ---
            e  = obj.get_e();      e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            w_meas_b = u.gyro';
            b_g = obj.get_b_g();
            w = w_meas_b - b_g;
            w0=w(1); w1=w(2); w2=w(3);

            b_a = obj.get_b_a();

            % body-frame specific force being rotated by q (same as yours)
            a_b = (u.accel' - b_a);
            a0 = a_b(1); a1 = a_b(2); a2 = a_b(3);

            % rotation used for dV/db_a block (same as yours)
            C_bi = ecef2body_rotm(e);     % body -> inertial
            C_EB = C_bi * a_b_mult;       % if gate false => zero

            J11 = obj.J(1,1);
            J22 = obj.J(2,2);
            J33 = obj.J(3,3);

            % =========================================================================
            % Pdot = V  => dPdot/dV = I
            % (this replaces your dx0dx/dx1dx/dx2dx rows)
            % =========================================================================
            A(P, V) = eye(3);

            % =========================================================================
            % Vdot Jacobian rows (this replaces dx3dx/dx4dx/dx5dx)
            % Your existing equations are exactly preserved.
            % =========================================================================

            % Row for d(V0)/dx  (your dx3dx)
            A(V(1), E)  = [ ...
                2*a0*e0 - 2*a1*e3 + 2*a2*e2, ...
                2*a0*e1 + 2*a1*e2 + 2*a2*e3, ...
                2*a1*e1 - 2*a0*e2 + 2*a2*e0, ...
                2*a2*e1 - 2*a0*e3 - 2*a1*e0 ...
                ];
            A(V(1), BA) = -C_EB(1,:);   % (-C_EB(1,:).')'  same content

            % Row for d(V1)/dx  (your dx4dx)
            A(V(2), E)  = [ ...
                2*a1*e0 + 2*a0*e3 - 2*a2*e1, ...
                2*a0*e2 - 2*a1*e1 - 2*a2*e0, ...
                2*a0*e1 + 2*a1*e2 + 2*a2*e3, ...
                2*a0*e0 - 2*a1*e3 + 2*a2*e2 ...
                ];
            A(V(2), BA) = -C_EB(2,:);

            % Row for d(V2)/dx  (your dx5dx)
            A(V(3), E)  = [ ...
                2*a1*e1 - 2*a0*e2 + 2*a2*e0, ...
                2*a1*e0 + 2*a0*e3 - 2*a2*e1, ...
                2*a1*e3 - 2*a0*e0 - 2*a2*e2, ...
                2*a0*e1 + 2*a1*e2 + 2*a2*e3 ...
                ];
            A(V(3), BA) = -C_EB(3,:);

            % =========================================================================
            % Quaternion kinematics Jacobian rows (dx6dx..dx9dx)
            % de/dt = -1/2 * Omega(w) * e
            % Your existing partials are preserved exactly as written.
            % =========================================================================

            % Row for de0/dx  (your dx6dx)
            A(E(1), E) = [0, -0.5*w0, -0.5*w1, -0.5*w2];
            % A(E(1), W) = [-0.5*e1, -0.5*e2, -0.5*e3];

            % Row for de1/dx  (your dx7dx)
            A(E(2), E) = [0.5*w0, 0, 0.5*w2, -0.5*w1];
            % A(E(2), W) = [0.5*e0, -0.5*e3, 0.5*e2];

            % Row for de2/dx  (your dx8dx)
            A(E(3), E) = [0.5*w1, -0.5*w2, 0, 0.5*w0];
            % A(E(3), W) = [0.5*e3, 0.5*e0, -0.5*e1];

            % Row for de3/dx  (your dx9dx)
            A(E(4), E) = [0.5*w2, 0.5*w1, -0.5*w0, 0];
            % A(E(4), W) = [-0.5*e2, 0.5*e1, 0.5*e0];

            % =========================================================================
            % Rigid body rotational dynamics Jacobian rows (dx10dx..dx12dx)
            % dw/dt = J^{-1}(-w x (Jw)) with your simplified partials
            % =========================================================================
            % Row for dw0/dx (your dx10dx)
            %{
            A(W(1), W) = [ ...
                0, ...
                w2 * (J22 - J33) / J11, ...
                w1 * (J22 - J33) / J11 ...
                ];

            % Row for dw1/dx (your dx11dx)
            A(W(2), W) = [ ...
                w2 * (J33 - J11) / J22, ...
                0, ...
                w0 * (J33 - J11) / J22 ...
                ];

            % Row for dw2/dx (your dx12dx)
            A(W(3), W) = [ ...
                w1 * (J11 - J22) / J33, ...
                w0 * (J11 - J22) / J33, ...
                0 ...
                ];
            %}
            % =========================================================================
            % Bias random walks: db/dt = 0  => rows already zero
            % (BG, BA, BP, and any extra states like b_m remain zero)
            % =========================================================================
        end

        function dxdt = f(obj, u)
            % State derivative using the SAME equations as your current code,
            % but assembled by named blocks for maintainability.

            % ---- Unpack state via getters (already index-safe) ----
            V_e = obj.get_V_E();     % 3x1
            e   = obj.get_e();       % 4x1
            % w_b = obj.get_w_b();     % 3x1

            % ---- Input handling ----
            a_meas_b = u.accel';
            w_meas_b = u.gyro';

            b_g = obj.get_b_g();

            % ---- Allocate output ----
            nx   = numel(obj.x_curr);
            dxdt = zeros(nx, 1);

            % ---- Kinematics / dynamics (same math) ----
            dP_dt = V_e;
            dV_dt = obj.calc_accel(a_meas_b);                         % C_bi*(a - b_a) + g_vec_e

            % Use gyro as input:
            w_corr = (w_meas_b - b_g);
            de_dt  = -0.5 * quat_kinematic_matrix(w_corr) * e;
            % dw_dt = obj.J \ (-cross(w_b, obj.J * w_b));               % rigid body (no torque)

            % ---- Bias models (same as your current: random walk / constant) ----
            db_g_dt = zeros(3,1);
            db_a_dt = zeros(3,1);
            db_p_dt = zeros(3,1);

            % If you later add mag bias b_m:
            % db_m_dt = zeros(3,1);

            % ---- Write into dxdt using indices (no hard-coded positions) ----
            dxdt(obj.x_inds.P_E) = dP_dt;
            dxdt(obj.x_inds.V_E) = dV_dt;
            dxdt(obj.x_inds.e)   = de_dt;
            % dxdt(obj.x_inds.w_b) = dw_dt;

            dxdt(obj.x_inds.b_g) = db_g_dt;
            dxdt(obj.x_inds.b_a) = db_a_dt;
            dxdt(obj.x_inds.b_p) = db_p_dt;

            % If you add b_m to x_inds, you just uncomment:
            % dxdt(obj.x_inds.b_m) = db_m_dt;

            % ---- Logging (keep exactly what you had) ----
            obj.accel_calc_all(obj.hist_idx, :) = dV_dt.';
        end

        function H = h_jacobian_states(obj, meas_idx)
            switch meas_idx
                case 1, H = obj.H_pos();
                case 2, H = obj.H_mag();
                % case 3, H = obj.H_gyro();
                case 3, H = obj.H_alt();
                otherwise, error("bad meas_idx");
            end
        end

        function y = h(obj, meas_idx)
            P_E = obj.get_P_E();
            b_p = obj.get_b_p();

            p_pred = P_E + b_p;

            % w_b = obj.get_w_b();
            b_g = obj.get_b_g();
            b_m = obj.get_b_m();

            % w_pred = w_b + b_g;

            alt_pred = obj.altitude();

            q = obj.get_e();

            % --- NEW: magnetometer prediction ---
            C_bi  = ecef2body_rotm(q);   % (given: this is b2i)
            C_ib  = C_bi.';              % i2b
            m_pred = C_ib * obj.m_ref_i + b_m; % predicted mag in body

            y_all = [
                p_pred;
                m_pred;
                % w_pred;
                alt_pred
                ];

            y = y_all(obj.measurement_ranges{meas_idx});
        end

        % --- JACOBIANS ---
        function H = H_pos(obj)
            H = zeros(3, obj.num_states);
            H(:, obj.x_inds.P_E) = eye(3);
            H(:, obj.x_inds.b_p) = eye(3);
        end

        function H = H_gyro(obj)
            H = zeros(3, obj.num_states);
            H(:, obj.x_inds.w_b) = eye(3);
            H(:, obj.x_inds.b_g) = eye(3);
        end

        %{
        function H = H_mag(obj)
            H = zeros(3, obj.num_states);

            q  = obj.get_e();
            m  = obj.m_ref_i;

            % analytic:
            Jq = obj.dC_times_m_dq_wxyz_i2b(q, m);   % 3x4

            H(:, obj.x_inds.e)   = Jq;
            H(:, obj.x_inds.b_m) = eye(3);
        end
        %}
        
        function H = H_mag(obj)
            H = zeros(3, obj.num_states);
            q  = obj.get_e();
            Jq = obj.dmag_dq_numeric(q);          % 3x4
            H(:, obj.x_inds.e)   = Jq;
            H(:, obj.x_inds.b_m) = eye(3);
        end

        function H = H_alt(obj)
            H = zeros(1, obj.num_states);
            H(:, obj.x_inds.P_E(3)) = 1;
        end

        % --- UTILS ---
        function normalize_quat(obj)
            obj.x_curr(obj.x_inds.e) = obj.get_e() / norm(obj.get_e());
        end

        function a_e = calc_accel(obj, a)
            e = obj.get_e();
            C_bi = ecef2body_rotm(e);          % body -> inertial

            b_a = obj.get_b_a();

            a_e  = C_bi * (a - b_a) + obj.g_vec_e;
        end

        function [q_meas, trust_accel] = quat_from_acc_mag(obj, a_b, m_b)
            a_norm = norm(a_b);
            trust_accel = (abs(a_norm - obj.g_norm) < obj.accel_gate) && (obj.altitude() < obj.alt_gate);

            if trust_accel
                d_b = -a_b / norm(a_b);
                obj.last_down = d_b;
            else
                d_b = obj.last_down;
            end

            obj.down_vec_all(obj.hist_idx, :) = d_b;

            u_b = -d_b;

            m_b = m_b / norm(m_b);

            e_b = cross(m_b, d_b);
            e_b = e_b / norm(e_b);

            n_b = cross(u_b, e_b);
            n_b = n_b / norm(n_b);

            C_BE = [ e_b, n_b, u_b ];

            C_bi  = C_BE.';                 % b2i
            q_meas = obj.rotm_to_quat(C_bi);
        end

        function q = rotm_to_quat(~, R)
            tr = trace(R);
            if tr > 0
                S  = sqrt(tr + 1.0) * 2.0;
                q0 = 0.25 * S;
                q1 = (R(3,2) - R(2,3)) / S;
                q2 = (R(1,3) - R(3,1)) / S;
                q3 = (R(2,1) - R(1,2)) / S;
            else
                if (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
                    S  = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2.0;
                    q0 = (R(3,2) - R(2,3)) / S;
                    q1 = 0.25 * S;
                    q2 = (R(1,2) + R(2,1)) / S;
                    q3 = (R(1,3) + R(3,1)) / S;
                elseif R(2,2) > R(3,3)
                    S  = sqrt(1.0 - R(1,1) + R(2,2) - R(3,3)) * 2.0;
                    q0 = (R(1,3) - R(3,1)) / S;
                    q1 = (R(1,2) + R(2,1)) / S;
                    q2 = 0.25 * S;
                    q3 = (R(2,3) + R(3,2)) / S;
                else
                    S  = sqrt(1.0 - R(1,1) - R(2,2) + R(3,3)) * 2.0;
                    q0 = (R(2,1) - R(1,2)) / S;
                    q1 = (R(1,3) + R(3,1)) / S;
                    q2 = (R(2,3) + R(3,2)) / S;
                    q3 = 0.25 * S;
                end
            end
            q = [q0; q1; q2; q3];
            q = q / norm(q);
        end

        function [ptr_out, rows] = take_rows_in_window_dir(~, time_vec, ptr_in, tmin, tmax, dir)
            % -------------------------------------------------------------------------
            % Helper: return indices in [tmin, tmax) and advance pointer, direction-aware.
            % Assumes time_vec is monotonic increasing (typical logged sensors).
            %
            % Forward (dir=+1): ptr points to next unread (starts at 1)
            % Backward (dir=-1): ptr points to next unread (starts at N)
            %
            % Returns rows in the pass order:
            %   forward: increasing time
            %   backward: decreasing time
            % -------------------------------------------------------------------------
            N = numel(time_vec);
            if N == 0
                ptr_out = ptr_in;
                rows = [];
                return;
            end

            if dir > 0
                p = ptr_in;

                % advance until we're at first time >= tmin
                while p <= N && time_vec(p) < tmin
                    p = p + 1;
                end

                start_p = p;

                % consume while time < tmax
                while p <= N && time_vec(p) < tmax
                    p = p + 1;
                end

                if start_p <= N && start_p < p
                    rows = start_p:(p-1);  % forward order
                else
                    rows = [];
                end

                ptr_out = p;

            else
                p = ptr_in;

                % move backward until we're at first time < tmax (since we're going down)
                while p >= 1 && time_vec(p) >= tmax
                    p = p - 1;
                end

                end_p = p;

                % consume while time >= tmin
                while p >= 1 && time_vec(p) >= tmin
                    p = p - 1;
                end

                start_p = p + 1;

                if start_p >= 1 && start_p <= end_p
                    rows = end_p:-1:start_p; % backward order (decreasing time)
                else
                    rows = [];
                end

                ptr_out = p; % next unread index when scanning backward
            end
        end

        function Jq = dmag_dq_numeric(obj, q)
            % Numeric Jacobian wrt q = [w x y z]' for:
            %   m_pred(q) = C_ib(q) * m_ref_i = ecef2body_rotm(q)' * m_ref_i
            % where ecef2body_rotm(q) returns C_bi (body->inertial, b2i).

            eps = 1e-6;

            q = q(:);
            q = q / norm(q);

            % baseline
            C_bi = ecef2body_rotm(q);      % b2i
            m0   = C_bi.' * obj.m_ref_i;   % i2b * m_ref_i

            Jq = zeros(3,4);

            for k = 1:4
                dq = zeros(4,1);
                dq(k) = eps;

                qp = q + dq;  qp = qp / norm(qp);
                qm = q - dq;  qm = qm / norm(qm);

                C_bi_p = ecef2body_rotm(qp);
                C_bi_m = ecef2body_rotm(qm);

                mp = C_bi_p.' * obj.m_ref_i;
                mm = C_bi_m.' * obj.m_ref_i;

                Jq(:,k) = (mp - mm) / (2*eps);
            end
        end

        function Jq = dC_times_m_dq_wxyz_i2b(~, q, m)
            % Analytic Jacobian of y = C_ib(q) * m wrt q = [w x y z]'.
            % Assumes C_ib is the "standard" i2b DCM for quaternion [w x y z].
            % Returns Jq (3x4): [dy/dw, dy/dx, dy/dy, dy/dz]

            q = q(:); m = m(:);
            q = q / norm(q);

            w = q(1); x = q(2); y = q(3); z = q(4);

            % These are partials of the STANDARD i2b DCM C_ib(q)
            dC_dw = [  0,  2*z, -2*y;
                -2*z,  0,  2*x;
                2*y, -2*x,  0 ];

            dC_dx = [  0,   2*y,  2*z;
                2*y, -4*x,  2*w;
                2*z, -2*w, -4*x ];

            dC_dy = [ -4*y,  2*x, -2*w;
                2*x,   0,   2*z;
                2*w,  2*z, -4*y ];

            dC_dz = [ -4*z,  2*w,  2*x;
                -2*w, -4*z,  2*y;
                2*x,  2*y,   0 ];

            % y = C*m  => dy/dq = (dC/dq)*m
            Jq = [dC_dw*m, dC_dx*m, dC_dy*m, dC_dz*m];
        end

    end

end
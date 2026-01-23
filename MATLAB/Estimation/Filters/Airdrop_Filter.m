classdef Airdrop_Filter < Abstract_Filter

    properties
        % --- Constants ---
        J
        g_norm      = 9.80665;
        accel_gate  = 1e0;
        alt_gate    = 1e10;
        speed_gate  = 1e100
        g_vec_e
        dt
        I

        % --- Measurement Matrices ---
        measurement_ranges

        dhdx_p
        dhdx_q
        dhdx_w
        dhdx_alt
        dhdx_w_no_bias
        dhdx_pos_no_bias

        % --- Filter Properties

        is_initialized

        x_curr
        P_curr

        H
        F

        update_a_b
        last_down
        last_u

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

            obj.measurement_ranges = {1:3, 4:7, 8:10, 11};

            obj.last_u = [0; 0; 0];
            obj.last_down = [0; 0; 1];
            obj.update_a_b = true;

            obj.init_x_inds();
            obj.init_matrices();

            obj.is_initialized = false;

            obj.hist_idx = 1;
        end

        % --- SETUP ---

        function init_x_inds(obj)
            obj.x_inds = struct(  ...
                'P_E', (1:3)', ... % Position in Inertial Frame
                'V_E', (4:6)', ... % Velocity in Inertial Frame
                'e',   (7:10)', ... % Quaternion
                'w_b', (11:13)', ... % Angular Velocity
                'b_g', (14:16)', ... % Gyro Bias
                'b_a', (17:19)', ... % Accelerometer Bias
                'b_p',  (20:22)' ... % GPS Position Bias
                );
        end

        function init_matrices(obj)
            obj.dhdx_p = [
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
                ];

            obj.dhdx_q = [
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                ];

            obj.dhdx_w = [
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0;
                ];

            obj.dhdx_alt = [
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                ];

            obj.dhdx_pos_no_bias = [
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                ];

            obj.dhdx_w_no_bias = [
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                ];
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

            % Change alt gate back
            obj.alt_gate = old_alt_gate;

            % Initialize velocity and angular velocity to zero
            obj.x_curr(obj.x_inds.V_E) = zeros(3,1);
            obj.x_curr(obj.x_inds.w_b) = zeros(3,1);

            % Initialize gyro bias to the current gyro readings
            obj.x_curr(obj.x_inds.b_g) = gyro_meas;
            % Initialize orientation
            obj.x_curr(obj.x_inds.e) = q_meas;
            % Initialize accel bias to the measurement minus corrected
            obj.x_curr(obj.x_inds.b_a) = accel_meas - accel_meas_corr;

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

                %{
                if t0 > drop_time
                    obj.R(4:7, 4:7) = (eye(4,4) * 1e-6) .^2;
                    obj.R(11, 11)   = 300;
                end
                %}

                % Define bin as [tmin, tmax)
                tmin = min(t0, t1);
                tmax = max(t0, t1);

                if verbose
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
                        obj.predict(u.data');
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
            if meas_idx == 2
                [y, trust_accel] = obj.quat_from_acc_mag(obj.last_u, y);

                
                if dot(y, obj.get_e()) < 0
                    y = -y;
                end
                
                
                obj.trust_accel_all(obj.hist_idx) = trust_accel;
                obj.quat_meas_all(obj.hist_idx, :)   = y;
            end

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

        function dfdx = f_jacobian_states(obj, u, a_b_mult)
            if nargin < 3
                a_b_mult = obj.update_a_b;
            end

            e = obj.get_e();
            e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            w_b = obj.get_w_b();
            w0 = w_b(1); w1 = w_b(2); w2 = w_b(3);

            a = obj.calc_accel(u(1:3));
            a0 = a(1); a1 = a(2); a2 = a(3);

            J11 = obj.J(1,1);
            J22 = obj.J(2,2);
            J33 = obj.J(3,3);
            
            C_EB = ecef2body_rotm(e)' * a_b_mult;

            dx0dx = [
                zeros(3,1);

                1;
                0;
                0;

                0;
                0;
                0;
                0;

                zeros(6,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx1dx = [
                zeros(3,1);

                0;
                1;
                0;

                0;
                0;
                0;
                0;

                zeros(6,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx2dx = [
                zeros(3,1);

                0;
                0;
                1;

                0;
                0;
                0;
                0;

                zeros(6,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx3dx = [
                zeros(3, 1);

                0;
                0;
                0;

                2*a0*e0 - 2*a1*e3 + 2*a2*e2;
                2*a0*e1 + 2*a1*e2 + 2*a2*e3;
                2*a1*e1 - 2*a0*e2 + 2*a2*e0;
                2*a2*e1 - 2*a0*e3 - 2*a1*e0;

                0;
                0;
                0

                0;
                0;
                0;

                -C_EB(1,:).';

                zeros(3,1);
                ]';

            dx4dx = [
                zeros(3, 1);
                0;
                0;
                0;

                2*a1*e0 + 2*a0*e3 - 2*a2*e1;
                2*a0*e2 - 2*a1*e1 - 2*a2*e0;
                2*a0*e1 + 2*a1*e2 + 2*a2*e3;
                2*a0*e0 - 2*a1*e3 + 2*a2*e2;

                0;
                0;
                0;

                0;
                0;
                0;


                -C_EB(2,:).';

                zeros(3,1);
                ]';


            dx5dx = [
                zeros(3, 1);
                0;
                0;
                0;

                2*a1*e1 - 2*a0*e2 + 2*a2*e0;
                2*a1*e0 + 2*a0*e3 - 2*a2*e1;
                2*a1*e3 - 2*a0*e0 - 2*a2*e2;
                2*a0*e1 + 2*a1*e2 + 2*a2*e3;

                0;
                0;
                0;

                0;
                0;
                0;

                -C_EB(3,:).';

                zeros(3,1);
                ]';

            dx6dx = [
                zeros(6, 1);

                0;
                -1/2*w0;
                -1/2*w1;
                -1/2*w2;

                -1/2*e1;
                -1/2*e2;
                -1/2*e3;

                zeros(3,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx7dx = [
                zeros(6, 1);

                1/2*w0;
                0;
                1/2*w2;
                -1/2*w1;

                1/2*e0;
                -1/2*e3;
                1/2*e2;

                zeros(3,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx8dx = [
                zeros(6, 1);

                1/2*w1;
                -1/2*w2;
                0;
                1/2*w0;

                1/2*e3;
                1/2*e0;
                -1/2*e1;

                zeros(3,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx9dx = [
                zeros(6, 1);

                1/2*w2;
                1/2*w1;
                -1/2*w0;
                0;

                -1/2*e2;
                1/2*e1;
                1/2*e0;

                zeros(3,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx10dx = [
                zeros(10, 1);
                0;
                w2 * (J22 - J33) / J11;
                w1 * (J22 - J33) / J11;

                zeros(3,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx11dx = [
                zeros(10, 1);
                w2 * (J33 - J11) / J22;
                0;
                w0 * (J33 - J11) / J22;

                zeros(3,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx12dx = [
                zeros(10, 1);
                w1 * (J11 - J22) / J33;
                w0 * (J11 - J22) / J33;
                0;

                zeros(3,1);
                zeros(3,1);
                zeros(3,1);
                ]';

            dx13dx = [
                zeros(13,1);
                0;
                0;
                0;
                zeros(3,1);
                zeros(3,1);
                ]';

            dx14dx = [
                zeros(13,1);
                0;
                0;
                0;
                zeros(3,1);
                zeros(3,1);
                ]';

            dx15dx = [zeros(13,1);
                0;
                0;
                0;
                zeros(3,1);
                zeros(3,1);
                ]';

            dx16dx = zeros(22, 1)';
            dx17dx = zeros(22, 1)';
            dx18dx = zeros(22, 1)';

            dx19dx = zeros(22, 1)';
            dx20dx = zeros(22, 1)';
            dx21dx = zeros(22, 1)';

            dfdx = [
                dx0dx;
                dx1dx;
                dx2dx;
                dx3dx;
                dx4dx;
                dx5dx;
                dx6dx;
                dx7dx;
                dx8dx;
                dx9dx;
                dx10dx;
                dx11dx;
                dx12dx;
                dx13dx;
                dx14dx;
                dx15dx;
                dx16dx;
                dx17dx;
                dx18dx;
                dx19dx;
                dx20dx;
                dx21dx
                ];
        end

        function dxdt = f(obj, u)
            V_e = obj.get_V_E();
            e   = obj.get_e();

            w_b = obj.get_w_b();

            dV_dt = obj.calc_accel(u(1:3));

            dP_dt = V_e;

            obj.accel_calc_all(obj.hist_idx, :) = dV_dt';

            de_dt = -1/2 * quat_kinematic_matrix(w_b) * e;
            dw_dt = obj.J \ (-cross(w_b, obj.J*w_b));

            db_g_dt = zeros(3,1);
            db_a_dt = zeros(3,1);
            db_p_dt = zeros(3,1);

            dxdt = [dP_dt; dV_dt; de_dt; dw_dt; db_g_dt; db_a_dt; db_p_dt];
        end

        function dhdx = h_jacobian_states(obj, meas_idx)
            w_jacobian = obj.dhdx_w;
            obj.update_a_b = true;

            if obj.altitude() > obj.alt_gate || obj.speed() > obj.speed_gate
                obj.update_a_b = false;
                w_jacobian = obj.dhdx_w_no_bias;
            end
            

            p_jacobian = obj.dhdx_p;

            dhdx = [
                p_jacobian
                obj.dhdx_q;
                w_jacobian;
                obj.dhdx_alt
                ];
            if nargin == 2
                dhdx = dhdx(obj.measurement_ranges{meas_idx}, :);
            end
        end

        function y = h(obj, meas_idx)
            P_E = obj.get_P_E();
            b_p = obj.get_b_p();

            p_pred = P_E + b_p;

            e_pred = obj.get_e();

            w_b = obj.get_w_b();
            b_g = obj.get_b_g();

            w_pred = w_b + b_g;

            alt_pred = obj.altitude();

            y_all = [
                p_pred;
                e_pred;
                w_pred;
                alt_pred
                ];

            y = y_all(obj.measurement_ranges{meas_idx});
        end

        % --- UTILS ---
        function normalize_quat(obj)
            obj.x_curr(obj.x_inds.e) = obj.get_e() / norm(obj.get_e());
        end

        function a_e = calc_accel(obj, a)
            e = obj.get_e();
            C_EB = ecef2body_rotm(e)';

            b_a = obj.get_b_a();

            a_e = C_EB * (a - b_a) + obj.g_vec_e;
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

            q_meas = obj.rotm_to_quat(C_BE);
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

    end

end
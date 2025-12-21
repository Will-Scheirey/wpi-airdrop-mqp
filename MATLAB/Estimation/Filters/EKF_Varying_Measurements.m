classdef EKF_Varying_Measurements < EKF_V_E
    properties
        measurement_ranges
        last_u

        g_norm = 9.80665;
        accel_gate  = 1;
        alt_gate    = 1e8;

        dhdx_alt
        dhdx_w_no_bias
        trust_accel_all
        dhdx_pos_no_bias
        down_vec_all

        last_down

        quat_meas_all
    end

    methods
        function obj = EKF_Varying_Measurements(R, Q, H0, P0, dt, J)
            x0 = zeros(height(P0), 1);

            obj = obj@EKF_V_E(R, Q, x0, H0, P0, dt, J);

            obj.measurement_ranges = {1:3, 4:7, 8:10, 11};
            obj.last_u = [0; 0; 0];

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

            obj.g_vec_e = [0; 0; -9.81];

            obj.last_down = [0; 0; 1];
            obj.update_a_b = true;
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
        end

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
            innovation = y - y_pred;

            H = obj.h_jacobian_states(meas_idx);

            range = obj.measurement_ranges{meas_idx};

            R_meas = obj.R(range, range);
            if nargin == 4
                R_meas = R_gps;
            end

            S = R_meas + H*obj.P_curr*H';

            K = obj.P_curr * H' / S;

            obj.x_curr = obj.x_curr + K * innovation;
            obj.normalize_quat();

            obj.P_curr = (obj.I - K*H) * obj.P_curr * (obj.I - K*H)' + K*R_meas*K';
        end

        function [innovation, S] = step_filter(obj, y, u)
            obj.predict(u);
            [innovation, ~, S] = obj.update(y);
        end

        function predict(obj, u)
            if isempty(u)
                u = obj.last_u;
            else
                obj.last_u = u;
            end

            predict@EKF_V_E(obj, u);
            obj.normalize_quat();
        end

        function run_filter(obj, y_all, u_all, timesteps, acc_gps_all)
            num_steps = numel(timesteps);
            dt = timesteps(2) - timesteps(1);

            obj.x_hist = zeros(numel(obj.x_curr), num_steps);
            obj.P_hist = zeros(numel(obj.x_curr), numel(obj.x_curr), num_steps);

            inno_height = 0;
            for i = 1:numel(y_all)
                inno_height = inno_height + width(y_all{i}.data);
            end

            obj.inno_hist = NaN(inno_height, num_steps);

            obj.S_hist = zeros(height(y_all), height(y_all), num_steps);

            obj.accel_calc_all = zeros(num_steps, 3);
            obj.trust_accel_all = NaN(num_steps, 1);
            obj.down_vec_all = NaN(num_steps, 3);
            obj.quat_meas_all = NaN(num_steps, 4);

            last_idx = zeros(size(y_all));
            last_idx_u = 0;

            for i=1:num_steps
                fprintf("%d / %d\n", i, num_steps)
                t = timesteps(i);
                obj.x_hist(:, i) = obj.x_curr;

                timestamps = u_all.time((last_idx_u + 1):end);

                above_min = t > timestamps;
                above_max = t + dt > timestamps;

                check = [above_min, above_max];

                good_meas = all(check, 2);

                if sum(good_meas) ~= 0
                    good_meas_idx = find(good_meas, 1);

                    u = u_all(good_meas_idx + last_idx_u, :);
                    last_idx_u = last_idx_u + good_meas_idx;
                    obj.predict(u.data');
                else
                    obj.predict([]);
                end

                for n = 1:numel(y_all)
                    y_all_n = y_all{n};

                    timestamps = y_all_n.time((last_idx(n) + 1):end);

                    above_min = t > timestamps;
                    below_max = t + dt > timestamps;

                    check = [above_min, below_max];

                    good_meas = all(check, 2);

                    if sum(good_meas) == 0
                        continue
                    end

                    good_meas_idx = find(good_meas, 1);

                    meas = y_all_n(good_meas_idx + last_idx(n), :);

                    last_idx(n) = last_idx(n) + good_meas_idx;

                    if isempty(meas)
                        continue
                    end

                    meas_idx = meas.meas_idx;

                    if meas_idx == 1
                        accH = acc_gps_all(last_idx(n), 1);
                        accV = acc_gps_all(last_idx(n), 2);
                        
                        R_gps = (blkdiag(accH, accH, accV)).^2;

                        [innovation, ~, S] = obj.update(meas.data', meas_idx, R_gps);
                    else
                        [innovation, ~, S] = obj.update(meas.data', meas_idx);
                    end

                    obj.inno_hist(obj.measurement_ranges{meas_idx}, obj.hist_idx) = innovation;
                end

                obj.P_hist(:, :, i) = obj.P_curr;
                obj.hist_idx = obj.hist_idx + 1;
            end
        end

        function dhdx = h_jacobian_states(obj, meas_idx)
            
            w_jacobian = obj.dhdx_w;

            if obj.x_curr(3) > obj.alt_gate || norm(obj.x_curr(4:6)) > 1
                w_jacobian = obj.dhdx_w_no_bias;
                obj.update_a_b = false;
            end
            
            % p_jacobian = obj.dhdx_pos_no_bias;
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

            bp0 = b_p(1); bp1 = b_p(2); bp2 = b_p(3);
            p0 = P_E(1); p1 = P_E(2); p2 = P_E(3);

            p = [p0+bp0; p1+bp1; p2+bp2];

            e = obj.get_e();
            e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            e = [e0; e1; e2; e3];

            w_b = obj.get_w_b();
            b_g = obj.get_b_g();

            bg0 = b_g(1); bg1 = b_g(2); bg2 = b_g(3);
            w0 = w_b(1); w1 = w_b(2); w2 = w_b(3);

            w = [w0 + bg0; w1 + bg1; w2 + bg2];

            alt = p2;

            y_all = [
                p;
                e;
                w;
                alt
                ];

            y = y_all(obj.measurement_ranges{meas_idx});
        end

        function [q_meas, trust_accel] = quat_from_acc_mag(obj, a_b, m_b)
            a_norm = norm(a_b);
            trust_accel = (abs(a_norm - obj.g_norm) < obj.accel_gate) && (obj.x_curr(3) < obj.alt_gate);

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
    end
end
classdef Airdrop_Filter < Abstract_Filter
    % AIRDROP_FILTER Implements matrix equations for 6-DOF state estimation
    %   This class implements all the equations for estimating the 6-DOF
    %   states of the payload during an airdrop. The class also has a
    %   function to run the filter through all of the measurements and
    %   inputs from onboard sensor/GPS data collected during an airdrop.
    %   
    properties
        % G_NORM Magnitude of constant gravitation acceleration [m/s^2]
        g_norm      = 9.80665;
        % G_VEC_E Gravitational acceleration vectory in -ENU [m/s^2]
        g_vec_e
        % DT Constant timestep for prediction
        dt
        % I Identity matrix for filter equations
        I

        % MEASUREMENT_RANGES Indices corresponding measurements
        measurement_ranges
        % MEAS_DEFS Struct of indices and dimensions for measurement names
        meas_defs
        
        % IS_INITIALIZED Whether the filter has been initialized yet
        is_initialized
        % NUM_STATES The number of states in the filter
        num_states

        % X_CURR The current state estimate colum vector
        x_curr
        % P_CURR The current state covariance estimate matrix
        P_curr

        % LAST_U The most recent inputs for the system
        last_u
        % M_REF_I The reference magnetic field vector
        m_ref_i

        % HIST_IDX The current index of historical data being saved
        hist_idx
        % ACCEL_CALC_ALL History of calculated inertial accel vectors
        accel_calc_all
        % P_HIST History of estimated state covariance matrices
        P_hist
        % X_HIST History of estimated states
        x_hist
        % S_HIST History of estimated innovation covariance
        S_hist
        % INNO_HIST History of measurement innovations
        inno_hist
    end

    methods (Abstract)
        % Abstract methods to be implemented by subclass
        update_impl(y, y_pred, H, R)
        predict_impl(u)
    end

    methods (Access = protected)
        % These methods are for if a subclass needs operations performed
        % before or after propagation steps

        function pre_step(obj, i)
            % Hook for subclasses. Default: no-op.
        end

        function post_step(obj, i)
            % Hook for subclasses. Default: no-op.
        end
    end

    methods
        function obj = Airdrop_Filter(R, Q, P0, dt)
            % AIRDROP_FILTER Creates an Aidrop_EKF object
            %
            % INPUTS: 
            %   R  : The initial measurement noise covariance matrix
            %   Q  : The initial process noise covariance matrix
            %   P0 : The initial state estimate covariance matrix
            %   dt : The constant timestep to use for propagation
            %
            % OUTPUTS:
            %   obj : The new Airdrop_Filter object

            obj.R = R;
            obj.Q = Q;

            obj.P_curr = P0;

            % Pre-allocate the identity matrix for future calculations
            obj.I = eye(size(P0));

            obj.dt = dt;

            % Set up the gravity vector (here it is positive because the
            % prediction equations have the gravity sign flipped. Using
            % proper sign convention, this would be negative)
            obj.g_vec_e = [0; 0; obj.g_norm];

            % Set the initial inputs to be 0
            obj.last_u = struct('accel', [0, 0, 0], 'gyro', [0, 0, 0]);

            % Initialize the state and measurement name indices
            obj.init_x_inds();
            obj.init_y_inds();

            % Set initialized to false
            obj.is_initialized = false;

            obj.hist_idx = 1;
        end

        % --- SETUP ---

        function init_x_inds(obj)
            % INIT_X_INDS Sets up the state name indices
            % 
            % INPUTS:
            %   obj : The Airdrop_Filter object

            % Define the state names and dimensions
            state_blocks = {
                "P_E", 3; ... % Position in Inertial Frame
                "V_E", 3; ... % Velocity in Inertial Frame
                "e",   4; ... % Quaternion
                "b_g", 3; ... % Gyro Bias
                "b_a", 3; ... % Accelerometer Bias
                "b_p", 3; ... % GPS Position Bias
                "b_m", 3; ... % Magnetometer Bias
                "b_b", 1; ... % Baro Bias
                "b_v", 3; ... % GPS Vel Bias
                };

            % Create the state name indices
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
            % INIT_Y_INDS Sets up the measurement name indices
            % 
            % INPUTS:
            %   obj : The Airdrop_Filter object

            % Define the measurement names, indices, and dimensions
            obj.meas_defs = struct( ...
                "pos", struct("idx",1,"dim",3), ...
                "mag", struct("idx",2,"dim",3), ...
                "alt", struct("idx",3,"dim",1), ...
                "vel", struct("idx",4,"dim",3) );

            r = 1;
            order = ["pos","mag","alt","vel"];
            for k = 1:numel(order)
                s = obj.meas_defs.(order(k));
                obj.measurement_ranges{s.idx} = r:(r+s.dim-1);
                r = r + s.dim;
            end
        end

        function initialize(obj, stationary, accel_meas, gyro_meas, mag_meas, gps_meas, baro_meas, q_meas, V_e)
            % INITIALIZE Initializes the filter for propagation
            %   This function initializes important properties like initial
            %   state estimates, the initial orientation, and the resulting
            %   reference magnetic field reading. 
            %
            % INPUTS:
            %   obj        : The Airdrop_Filter object
            %   stationary : If the object is stationary for initialization
            %   accel_meas : The initial acceleration measurement
            %   gyro_meas  : The initial gyroscope measurement
            %   mag_meas   : The initial magnetometer measurement
            %   gps_meas   : The initial GPS position measurement
            %   baro_meas  : The initial barometer altitude measurement
            %   q_meas     : (Optional) The initial quaternion
            %   V_e        : (Optional) The initial velocity
            %
            
            % Create the state estimate vector
            obj.x_curr = zeros(obj.num_states, 1);

            pos_inds = obj.x_inds.P_E;
            % Initialize position to the GPS measurement
            obj.x_curr(pos_inds(1:2)) = gps_meas(1:2);

            % Initialize altitude to the baro measurement
            obj.x_curr(pos_inds(3))   = baro_meas;

            % Initialize altitude bias to GPS altitude minus baro altitude
            obj.x_curr(obj.x_inds.b_p(3)) = gps_meas(3) - baro_meas;

            % If we are not stationary, we can't do anything else
            if ~stationary, return; end 

            % Rescale accel to have the expected norm
            accel_meas_corr = obj.g_norm * accel_meas / norm(accel_meas);

            % Estimate the orientation from accel and mag measurments
            if nargin < 8
                q_meas = quat_from_acc_mag(accel_meas_corr, mag_meas);
            end

            % Create our reference magnetic field vectr
            C_bi = body2enu_rotm(q_meas);
            m_b0 = mag_meas / norm(mag_meas);  % measured mag in body

            obj.m_ref_i = C_bi * m_b0;         % inertial reference mag

            % Initialize velocity and angular velocity to zero
            if nargin < 9, V_e = zeros(3,1); end
            obj.x_curr(obj.x_inds.V_E) = V_e;

            % Initialize gyro bias to the current gyro readings
            obj.x_curr(obj.x_inds.b_g) = gyro_meas;

            % Initialize orientation
            obj.x_curr(obj.x_inds.e) = q_meas;
            
            % Initialize accel bias to the measurement minus corrected
            obj.x_curr(obj.x_inds.b_a) = accel_meas - accel_meas_corr;

            % Make sure we have a column vector
            obj.x_curr = obj.x_curr(:);

            % Done initializing!
            obj.is_initialized = true;

            % This internal function generates the NED quaternion
            function q_meas = quat_from_acc_mag(a_b, m_b)
                d_b = -a_b / norm(a_b);

                u_b = -d_b;

                m_b = m_b / norm(m_b);

                e_b = cross(m_b, d_b);
                e_b = e_b / norm(e_b);

                n_b = cross(u_b, e_b);
                n_b = n_b / norm(n_b);

                C_BE = [ e_b, n_b, u_b ];

                C_bi  = C_BE.';                 % b2i
                q_meas = rotm_to_quat(C_bi);
            end
        end

        % --- GETTERS ---
        % These are just utility functions to get states
        function P_E_out = get_P_E(obj)
            P_E_out = obj.x_curr(obj.x_inds.P_E);
        end

        function V_E_out = get_V_E(obj)
            V_E_out = obj.x_curr(obj.x_inds.V_E);
        end

        function e_out = get_e(obj)
            e_out   = obj.x_curr(obj.x_inds.e);
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

        function b_b_out = get_b_b(obj)
            b_b_out = obj.x_curr(obj.x_inds.b_b);
        end

        function b_v_out = get_b_v(obj)
            b_v_out = obj.x_curr(obj.x_inds.b_v);
        end

        function alt_out = altitude(obj)
            pos = obj.get_P_E();
            alt_out = pos(3);
        end

        function speed_out = speed(obj)
            speed_out = norm(obj.get_V_E());
        end

        function run_filter(obj, y_all, u_all, timesteps, acc_gps_all, drop_time, direction, verbose, order)
            % RUN_FILTER Run filter forward/backward with multiple async measurement streams.
            %   PLEASE NOTE: This function was generated with ChatGPT. It
            %   has been reviewed by a human for functionality, but has not
            %   been modified and is assumed to work. As such, the code is
            %   also not documented.
            %
            % direction: +1 for forward, -1 for backward
            % order: "predict_then_update" (default) or "update_then_predict"
            %
            % INPUTS:
            %   obj         : The Aidrop_Filter object
            %   y_all       : The table of all measurements
            %   u_all       : The table of all inputs
            %   timestamps  : The timestamps to output predictions for
            %   acc_gps_all : List of all GPS accuracy estimates
            %       NOTE: This is not currently used
            %   drop_time   : The timestamp when the airdrop occurred
            %   direction   : The direction to run the filter (+ or -)
            %   verbose     : Whether to output debug information
            %   order       : Whether to predict first or update first
            %       Either "predict_then_update" or "update_then_predict"

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

                % Define bin as [tmin, tmax)
                tmin = min(t0, t1);
                tmax = max(t0, t1);

                if verbose && mod(ii, 1) == 0
                    fprintf("%d / %d (t=%.6f)\n", ii, num_steps, t0);
                end

                if t0 > drop_time
                    % obj.Q(obj.x_inds.b_m, obj.x_inds.b_m) = eye(3) * 1e-8;
                    % obj.Q(obj.x_inds.b_a, obj.x_inds.b_a) = eye(3) * 1e-4;
                    % obj.Q(obj.x_inds.b_g, obj.x_inds.b_g) = eye(3) * 1e-4;
                end

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

                        %{
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
                        %}
                        [innovation, ~, S] = obj.update(meas.data', meas_idx);

                        obj.inno_hist(obj.measurement_ranges{meas_idx}, obj.hist_idx) = innovation;
                        S_list{kk} = S;
                    end

                    obj.S_hist{step_i, n} = S_list;
                end
            end
        end

        % --- FILTER MATH ---

        function [innovation, K, S] = update(obj, y, meas_idx, R_gps)
            % UPDATE Runs the update step for the filter
            %
            % INPUTS:
            %   obj      : The Aidrop_EKF object
            %   y        : The measurement for this update
            %   meas_idx : The index this measurement corresponds to
            %   R_gps    : GPS variance (not currently used)
            %
            % OUTPUTS:
            %   innovation : The innovation for this update step
            %   K          : The Kalman gain matrix
            %   S          : The innovation covariance estimate

            % Generate the prediction and its jacobian
            y_pred = obj.h(meas_idx);
            H = obj.h_jacobian_states(meas_idx);

            % Extract the right measurement nosie covariance
            range = obj.measurement_ranges{meas_idx};
            R_meas = obj.R(range, range);

            % Run the update implementation
            [innovation, K, S] = obj.update_impl(y, y_pred, H, R_meas);

            % Normalize the quaternion 
            obj.normalize_quat();
        end

        function predict(obj, u)
            % PREDICT Runs the predict step for the filter
            %   This function runs the predict step with the current
            %   constant timestep; if no input exists for this current
            %   timestep, the last input will be used.
            %   
            % INPUTS:
            %   obj : The Aidrop_Filter object
            %   u   : The IMU inputs for this step
            %       Includes both acceleration and gyroscope measurements

            % Make sure we have an input to use
            if isempty(u)
                u = obj.last_u;
            else
                obj.last_u = u;
            end

            % Run the prediction implementation
            obj.predict_impl(u);

            % Normalize the quaternion
            obj.normalize_quat();
        end

        function A = f_jacobian_states(obj, u)
            % F_JACOBIAN_STATES Calculates the state derivative Jacobian
            % 
            % INPUTS:
            %   obj : The Aidrop_Filter object
            %   u   : The inputs for this step
            %
            % OUTPUTS:
            %   A : The Jacobian
            
            % Extract the inputs (must be a table or a struct)
            a = u.accel(:);     a0  = a(1);  a1  = a(2);  a2  = a(3);
            w = u.gyro(:);      w0  = w(1);  w1  = w(2);  w2  = w(3);

            % Get necessary states
            e = obj.get_e();    e0  = e(1);  e1  = e(2);  e2  = e(3);  e3 = e(4); 
            ba = obj.get_b_a(); ba0 = ba(1); ba1 = ba(2); ba2 = ba(3);
            bw = obj.get_b_g(); bw0 = bw(1); bw1 = bw(2); bw2 = bw(3);
            
            % Hardcoded Jacobian (could write this neater, but don't want
            % to risk copying wrong)
            A = [
                0, 0, 0, 1, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 1, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 1,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 2*e0*(a0 - ba0) - 2*e3*(a1 - ba1) + 2*e2*(a2 - ba2), 2*e1*(a0 - ba0) + 2*e2*(a1 - ba1) + 2*e3*(a2 - ba2), 2*e1*(a1 - ba1) - 2*e2*(a0 - ba0) + 2*e0*(a2 - ba2), 2*e1*(a2 - ba2) - 2*e3*(a0 - ba0) - 2*e0*(a1 - ba1),     0,     0,     0, - e0^2 - e1^2 + e2^2 + e3^2,           2*e0*e3 - 2*e1*e2,         - 2*e0*e2 - 2*e1*e3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 2*e0*(a1 - ba1) + 2*e3*(a0 - ba0) - 2*e1*(a2 - ba2), 2*e2*(a0 - ba0) - 2*e1*(a1 - ba1) - 2*e0*(a2 - ba2), 2*e1*(a0 - ba0) + 2*e2*(a1 - ba1) + 2*e3*(a2 - ba2), 2*e0*(a0 - ba0) - 2*e3*(a1 - ba1) + 2*e2*(a2 - ba2),     0,     0,     0,         - 2*e0*e3 - 2*e1*e2, - e0^2 + e1^2 - e2^2 + e3^2,           2*e0*e1 - 2*e2*e3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 2*e1*(a1 - ba1) - 2*e2*(a0 - ba0) + 2*e0*(a2 - ba2), 2*e0*(a1 - ba1) + 2*e3*(a0 - ba0) - 2*e1*(a2 - ba2), 2*e3*(a1 - ba1) - 2*e0*(a0 - ba0) - 2*e2*(a2 - ba2), 2*e1*(a0 - ba0) + 2*e2*(a1 - ba1) + 2*e3*(a2 - ba2),     0,     0,     0,           2*e0*e2 - 2*e1*e3,         - 2*e0*e1 - 2*e2*e3, - e0^2 + e1^2 + e2^2 - e3^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                        bw0/2 - w0/2,                                        bw1/2 - w1/2,                                        bw2/2 - w2/2,  e1/2,  e2/2,  e3/2,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                        w0/2 - bw0/2,                                                   0,                                        w2/2 - bw2/2,                                        bw1/2 - w1/2, -e0/2,  e3/2, -e2/2,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                        w1/2 - bw1/2,                                        bw2/2 - w2/2,                                                   0,                                        w0/2 - bw0/2, -e3/2, -e0/2,  e1/2,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                        w2/2 - bw2/2,                                        w1/2 - bw1/2,                                        bw0/2 - w0/2,                                                   0,  e2/2, -e1/2, -e0/2,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0,                                                   0,                                                   0,                                                   0,                                                   0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                ];
        end

        function dxdt = f(obj, u)
            % F Calculates the state derivatives
            % 
            % INPUTS:
            %   obj : The Aidrop_Filter object
            %   u   : The inputs for this step
            %
            % OUTPUTS:
            %   dxdt : The state derivative vector

            % Get relevant states
            v = obj.get_V_E();  v0  = v(1);  v1  = v(2);  v2  = v(3);
            a = u.accel(:);     a0  = a(1);  a1  = a(2);  a2  = a(3);
            w = u.gyro(:);      w0  = w(1);  w1  = w(2);  w2  = w(3);
            e = obj.get_e();    e0  = e(1);  e1  = e(2);  e2  = e(3);  e3 = e(4); 
            ba = obj.get_b_a(); ba0 = ba(1); ba1 = ba(2); ba2 = ba(3);
            bw = obj.get_b_g(); bw0 = bw(1); bw1 = bw(2); bw2 = bw(3);
            g = obj.g_vec_e;  g0  = g(1);  g1  = g(2);  g2  = g(3);

            % Hardcoded state derivative vector
            dxdt = [
            v0;
            v1;
            v2;
            (a0 - ba0)*(e0^2 + e1^2 - e2^2 - e3^2) - g0 - (a1 - ba1)*(2*e0*e3 - 2*e1*e2) + (a2 - ba2)*(2*e0*e2 + 2*e1*e3);
            (a1 - ba1)*(e0^2 - e1^2 + e2^2 - e3^2) - g1 + (a0 - ba0)*(2*e0*e3 + 2*e1*e2) - (a2 - ba2)*(2*e0*e1 - 2*e2*e3);
            (a2 - ba2)*(e0^2 - e1^2 - e2^2 + e3^2) - g2 - (a0 - ba0)*(2*e0*e2 - 2*e1*e3) + (a1 - ba1)*(2*e0*e1 + 2*e2*e3);
            (e1*(bw0 - w0))/2 + (e2*(bw1 - w1))/2 + (e3*(bw2 - w2))/2;
            (e3*(bw1 - w1))/2 - (e0*(bw0 - w0))/2 - (e2*(bw2 - w2))/2;
            (e1*(bw2 - w2))/2 - (e3*(bw0 - w0))/2 - (e0*(bw1 - w1))/2;
            (e2*(bw0 - w0))/2 - (e1*(bw1 - w1))/2 - (e0*(bw2 - w2))/2;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            0;
            ];
        end

        function H = h_jacobian_states(obj, meas_idx)
            % H_JACOBIAN_STATES Calculates the measurement Jacobian
            % 
            % INPUTS:
            %   obj        : The Aidrop_Filter object
            %   meas_idx   : The corresponding index for this measurement
            %
            % OUTPUTS:
            %   H : The Jacobian

            % Find the measurement indices
            switch meas_idx
                case 1, range = 1:3;
                case 2, range = 4:6;
                case 3, range = 7;
                case 4, range = 8:10;
                otherwise, error("bad meas_idx");
            end

            % Get the relevant states
            v = obj.get_V_E();  v0  = v(1);  v1  = v(2);  v2  = v(3);
            e = obj.get_e();    e0  = e(1);  e1  = e(2);  e2  = e(3);  e3 = e(4); 
            ba = obj.get_b_a(); ba0 = ba(1); ba1 = ba(2); ba2 = ba(3);
            bw = obj.get_b_g(); bw0 = bw(1); bw1 = bw(2); bw2 = bw(3);
            g = obj.g_vec_e;  g0  = g(1);  g1  = g(2);  g2  = g(3);
            m0 = obj.m_ref_i; m00 = m0(1); m01 = m0(2); m02 = m0(3);

            % Hardcoded Jacobian (could write this neater, but don't want
            % to risk copying wrong)
            H = [
            1, 0, 0, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0;
            0, 1, 0, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0;
            0, 0, 1, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0;
            0, 0, 0, 0, 0, 0, 2*e0*m00 - 2*e2*m02 + 2*e3*m01, 2*e1*m00 + 2*e2*m01 + 2*e3*m02, 2*e1*m01 - 2*e0*m02 - 2*e2*m00, 2*e0*m01 + 2*e1*m02 - 2*e3*m00, 0, 0, 0, 0, 0, 0,  0,  0,  0, -1,  0,  0,  0,  0,  0,  0;
            0, 0, 0, 0, 0, 0, 2*e0*m01 + 2*e1*m02 - 2*e3*m00, 2*e0*m02 - 2*e1*m01 + 2*e2*m00, 2*e1*m00 + 2*e2*m01 + 2*e3*m02, 2*e2*m02 - 2*e0*m00 - 2*e3*m01, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0, -1,  0,  0,  0,  0,  0;
            0, 0, 0, 0, 0, 0, 2*e0*m02 - 2*e1*m01 + 2*e2*m00, 2*e3*m00 - 2*e1*m02 - 2*e0*m01, 2*e0*m00 - 2*e2*m02 + 2*e3*m01, 2*e1*m00 + 2*e2*m01 + 2*e3*m02, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0, -1,  0,  0,  0,  0;
            0, 0, 1, 0, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0, -1,  0,  0,  0;
            0, 0, 0, 1, 0, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0, -1,  0,  0;
            0, 0, 0, 0, 1, 0,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0, -1,  0;
            0, 0, 0, 0, 0, 1,                              0,                              0,                              0,                              0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1;
            ];

            % Extract only the prediction for this measurement
            H = H(range, :);
        end

        function y = h(obj, meas_idx)
            % H Calculates the measurement prediction
            % 
            % INPUTS:
            %   obj        : The Aidrop_Filter object
            %   meas_idx   : The corresponding index for this measurement
            %
            % OUTPUTS:
            %   y : The prediction

            % Get relevant states
            p = obj.get_P_E();  p0  = p(1);  p1  = p(2);  p2  = p(3);
            v = obj.get_V_E();  v0  = v(1);  v1  = v(2);  v2  = v(3);
            e = obj.get_e();    e0  = e(1);  e1  = e(2);  e2  = e(3);  e3 = e(4); 
            bp = obj.get_b_p(); bp0 = bp(1); bp1 = bp(2); bp2 = bp(3);
            bv = obj.get_b_v(); bv0 = bv(1); bv1 = bv(2); bv2 = bv(3);
            bm = obj.get_b_m(); bm0 = bm(1); bm1 = bm(2); bm2 = bm(3);
            bb = obj.get_b_b();
            ba = obj.get_b_a(); ba0 = ba(1); ba1 = ba(2); ba2 = ba(3);
            bw = obj.get_b_g(); bw0 = bw(1); bw1 = bw(2); bw2 = bw(3);
            g = obj.g_vec_e;  g0  = g(1);  g1  = g(2);  g2  = g(3);
            m0 = obj.m_ref_i; m00 = m0(1); m01 = m0(2); m02 = m0(3);

            % The prediction
            y_all = [
            p0 - bp0;
            p1 - bp1;
            p2 - bp2;
            % Rotate the reference inertial magnetic field into the body frame 
            m01*(2*e0*e3 + 2*e1*e2) - bm0 - m02*(2*e0*e2 - 2*e1*e3) + m00*(e0^2 + e1^2 - e2^2 - e3^2);
            m02*(2*e0*e1 + 2*e2*e3) - m00*(2*e0*e3 - 2*e1*e2) - bm1 + m01*(e0^2 - e1^2 + e2^2 - e3^2);
            m00*(2*e0*e2 + 2*e1*e3) - bm2 - m01*(2*e0*e1 - 2*e2*e3) + m02*(e0^2 - e1^2 - e2^2 + e3^2);
            p2 - bb;
            v0 - bv0;
            v1 - bv1;
            v2 - bv2;
            ];
            
            % Extract only the prediction for this measurement
            y = y_all(obj.measurement_ranges{meas_idx});
        end

        % --- UTILS ---
        function normalize_quat(obj)
            % NORMALIZE_QUAT Normalizes the quaternion
            % 
            % INPUTS:
            %   obj : The Aidrop_Filter object
            %

            obj.x_curr(obj.x_inds.e) = obj.get_e() / norm(obj.get_e());
        end

        function [ptr_out, rows] = take_rows_in_window_dir(~, time_vec, ptr_in, tmin, tmax, dir)
            % NOTE: This function was generated by ChatGPT. It has been
            % reviewed for functionality, but not modified or documented.
            
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
classdef Parachute_Model_Simple < Dynamic_Model
    %PARACHUTE_MODEL_SIMPLE Implementation of the basic parachute model
    %   This class implements the dynamical model for the 12-DOF
    %   payload-parachute system, connected to one another by a
    %   spring-damper linkage riser
    %
    %   "Parachute" and "Canopy" are used interchangeably here; the suffix
    %   "c" is used for the parachute, meaning "canopy", since the suffix
    %   "p" is used for the payload
    %
    %   The state vector x has the following structure:
    %       P_p : (3x1) (1:3)   Payload position         (inertial)
    %       V_p : (3x1) (4:6)   Payload velocity         (body)
    %       e_p : (4x1) (7:10)  Payload quaternion
    %       w_p : (3x1) (11:13) Payload angular velocity (body)
    %       P_c : (3x1) (14:16) Canopy  position         (inertial)
    %       V_c : (3x1) (17:19) Canopy  velocity         (body)
    %       e_c : (4x1) (20:23) Canopy  quaternion
    %       w_c : (3x1) (24:26) Canopy  angular velocity (body)

    properties
        % PARACHUTE The parachute object for the model
        parachute
        % PAYLOAD The payload object for the model
        payload

        % M_PAYLOAD Payload mass for the current timestep
        m_payload
        % M_PARACHUTE Parachute mass for the current timestep
        m_parachute

        % P_P Payload position for the current timestep
        P_p
        % P_C Canopy position for the current timestep
        P_c

        % E_P Payload orientation quaternion for the current timestep
        e_p
        % E_C Canopy orientation quaternion for the current timestep
        e_c

        % V_P Payload velocity for the current timestep
        V_p
        % V_C Canopy velocity for the current timestep
        V_c

        % W_P Payload angular velocity for the current timestep
        w_p
        % W_C Canopy angular velocity for the current timestep
        w_c

        % C_EB_P Payload Body->Inertial rotation matrix for the current timestep
        C_EB_p
        % C_EB_C Canopy Body->Inertial rotation matrix for the current timestep
        C_EB_c

        % RHO Air density for the current timestep, calculated using the
        % standard atmosphere model
        rho

        % DRAG_FORCE_P Drag force on the payload for the current timestep
        drag_force_p
        % DRAG_FORCE_C Drag force on the canopy for the current timestep
        drag_force_c

        % LIFT_FORCE_P Lift force on the payload for the current timestep
        lift_force_p
        % LIFT_FORCE_C Lift force on the canopy for the current timestep
        lift_force_c

        % A_P_CURR Acceleration of the payload for the current timestep
        a_p_curr
        % A_C_CURR Acceleration of the canopy for the current timestep
        a_c_curr

        % ALPHA_P_CUUR Angular acceleration of the payload for the current timestep
        alpha_p_curr
        % ALPHA_C_CUUR Angular acceleration of the canopy for the current timestep
        alpha_c_curr

        % AOA_P_CURR Angle of attack of the payload for the current timestep
        aoa_p_curr
        % AOA_C_CUUR Angle of attack of the canopy for the current timestep
        aoa_c_curr
    end

    methods
        function obj = Parachute_Model_Simple(payload, parachute, x0)
            % PARACHUTE_MODEL_SIMPLE Construct an instance of this class
            %
            % INPUTS:
            %   payload   : The Payload object for the simulation
            %   parachute : The Parachute object for the simulation
            %   x0        : The initial state vector for the system
            %
            % OUTPUTS:
            %   obj : The new Parachute_Model_Simple object

            obj = obj@Dynamic_Model(x0);

            obj.parachute = parachute;
            obj.payload   = payload;
        end

        function x_dot = ode_fcn(obj, ~, x)
            % ODE_FCN The ODE function, called by the ODE solver
            %
            % INPUTS:
            %   obj : The Parachute_Model_Simple
            %   t   : The timestep
            %   x   : The current state vector
            %
            % OUTPUTS:
            %   x_dot : The calculated state derivative vector

            obj.get_states(x);

            [F_p, F_c, M_p, M_c] = obj.equations_of_motion();

            x_dot = obj.dxdt(F_p, F_c, M_p, M_c);
        end

        function [F_p, F_c, M_p, M_c] = equations_of_motion(obj)
            % EQUATIONS_OF_MOTION Calculates the motion for the objects
            %   This function calculates forces and moments for the payload
            %   and parachute based on the current states. The current
            %   states are assumed to be set and updated as class
            %   properties
            %
            % INPUTS:
            %   obj : The Parachute_Model_Simple object
            %
            % OUTPUTS:
            %   F_p : The calculated force on the payload
            %   F_c : The calculated force on the canopy
            %   M_p : The calculated moment on the payload
            %   M_c : The calculated moment on the canopy


            % --- Body Forces ---
            [F_g_p, F_g_c] = obj.calc_gravity();
            [F_d_p, F_d_c] = obj.calc_drag();
            [F_r_p, F_r_c] = obj.calc_riser_force();

            F_p = F_g_p + F_d_p + F_r_p; % Resultant payload force [N]
            F_c = F_g_c + F_d_c + F_r_c; % Resultant canopy  force [N]

            % --- Body Moments ---
            M_p = cross(obj.payload.  P_attach_B, F_r_p); % Payload [N m]
            M_c = cross(obj.parachute.P_attach_B, F_r_c); % Canopy  [N m]
        end

        function x_dot = dxdt(obj, F_p, F_c, M_p, M_c)
            % DXDT Calculates the state derivatives for the timestep
            %   This function calculates the state derivatives based on the
            %   current states, forces, and moments, assuming the current
            %   states exist and have been already set as object properties
            %
            % INPUTS:
            %   F_p : Force on the payload
            %   F_c : Force on the canopy
            %   M_p : Moment on the payload
            %   M_c : Moment on the canopy
            %
            % OUTPUTS:
            %   x_dot : The state derivative vector

            % ===========================
            % --- Kinematics ---
            % ===========================

            % Calculate translational and rotational accelerations
            [a_p, alpha_p] = obj.calc_accel(obj.V_p, obj.w_p, obj.m_payload,   obj.payload.I(obj.rho),   F_p, M_p);
            [a_c, alpha_c] = obj.calc_accel(obj.V_c, obj.w_c, obj.m_parachute, obj.parachute.I(obj.rho), F_c, M_c);

            % For future extraction
            obj.a_p_curr = a_p;
            obj.a_c_curr = a_c;

            obj.alpha_p_curr = alpha_p;
            obj.alpha_c_curr = alpha_c;

            % Quaternion kinematics
            e_p_dot   = -1/2 * quat_kinematic_matrix(obj.w_p) * obj.e_p; % Quaternion rates
            e_c_dot   = -1/2 * quat_kinematic_matrix(obj.w_c) * obj.e_c; % Quaternion rates

            % ==============
            % --- States ---
            % ==============
            x_dot = [
                obj.C_EB_p' * obj.V_p; % Rotate body velocity into inertial
                a_p;
                e_p_dot
                alpha_p;

                obj.C_EB_c' * obj.V_c; % Rotate body velocity into inertial
                a_c;
                e_c_dot;
                alpha_c;
                ];
        end

        function get_states(obj, x)
            % GET_STATES Sets the current states as object properties
            %
            % INPUTS:
            %   obj : The Airdrop_Model_Simple object
            %   x   : The current state vector

            % --- Payload ---
            obj.P_p = x(1:3);
            obj.V_p = x(4:6);

            obj.e_p = x(7:10) / norm(x(7:10)); % Normalize quaternion
            obj.w_p = x(11:13);

            % --- Parachute ---
            obj.P_c = x(14:16);
            obj.V_c = x(17:19);

            obj.e_c = x(20:23) / norm(x(20:23)); % Normalize quaternion
            obj.w_c = x(24:26);

            % --- Additional States ---

            % Air density at altitude
            obj.rho = StandardAtmosphereModel.Density(obj.P_p(3));

            % Parachute mass parameters can change based on air density.
            % The accumulated air inside the parachute will change density
            % based on air pressure, changing the effective parachute mass.
            % While the payload mass is constant, mass is still retrieved
            % through a function
            obj.m_payload   = obj.payload.  m(obj.rho);
            obj.m_parachute = obj.parachute.m(obj.rho);

            % --- Rotations ---
            obj.C_EB_p = body2enu_rotm(obj.e_p)'; % ROTM from ECEF to Body
            obj.C_EB_c = body2enu_rotm(obj.e_c)'; % ROTM from ECEF to Body
        end

        function [f_p, f_c] = calc_gravity(obj)
            % CALC_GRAVITY Calculates body gravity force for the objects
            %
            % INPUTS:
            %   obj : The Parachute_Model_Simple object
            %
            % OUTPUTS:
            %   f_p : Gravity force on the payload
            %   f_c : Gravity force on the canopy

            % Rotate the inertial gravity into the body frame
            f_p = obj.C_EB_p * obj.g_vec_e * obj.m_payload;
            f_c = obj.C_EB_c * obj.g_vec_e * obj.m_parachute;
        end

        function [f_p, f_c] = calc_drag(obj)
            % CALC_DRAG Calculates drag forces on the objects
            %
            % INPUTS:
            %   obj : The Parachute_Model_Simple object
            %
            % OUTPUTS:
            %   f_p : Drag force on the payload
            %   f_c : Drag force on the canopy
            %

            % Get angle of attack
            aoa_p = flight_angles(obj.V_p, obj.C_EB_p);
            aoa_c = flight_angles(obj.V_c, obj.C_EB_c);

            % Calculate drag force, assuming no wind
            f_p = -0.5 * obj.rho * obj.payload.CdS(aoa_p)   * obj.V_p * norm(obj.V_p);
            f_c = -0.5 * obj.rho * obj.parachute.CdS(aoa_c) * obj.V_c * norm(obj.V_c);

            % If velocity is zero, normalizing will give nan
            if any(isnan(f_p)), f_p = [0; 0; 0]; end
            if any(isnan(f_c)), f_c = [0; 0; 0]; end

            obj.aoa_p_curr = aoa_p;
            obj.aoa_c_curr = aoa_c;

            obj.drag_force_p = f_p;
            obj.drag_force_c = f_c;

            % ==============================
            % --- Parachute Lift         ---
            % ==============================
            % Implement lift using the convention from pioneer slides
            %   D = cos(aoa)*F  L = sin(aoa)*F  so,  L = tan(aoa)*D
            %
            % Lift direction is chosen as the component opposite gravity
            % that is perpendicular to the relative wind (velocity).

            f_l_p = [0; 0; 0];
            f_l_c = [0; 0; 0];

            v_mag_c = norm(obj.V_c);
            if v_mag_c > 1e-6
                v_hat_c = obj.V_c / v_mag_c;

                % "Up" direction expressed in canopy body frame
                g_b_c = obj.C_EB_c * obj.g_vec_e;
                g_up_b_c = -g_b_c / norm(g_b_c);

                % Lift direction: component of up-direction perpendicular to relative wind
                lift_dir_c = g_up_b_c - dot(g_up_b_c, v_hat_c) * v_hat_c;

                if norm(lift_dir_c) < 1e-6
                    % fallback if v is nearly aligned with gravity
                    lift_dir_c = cross(v_hat_c, [0; 1; 0]);
                    if norm(lift_dir_c) < 1e-6
                        lift_dir_c = cross(v_hat_c, [1; 0; 0]);
                    end
                end

                lift_dir_c = lift_dir_c / norm(lift_dir_c);

                % Lift magnitude based on drag magnitude
                v_rel_c_e = obj.C_EB_c' * obj.V_c;
                Vh = norm(v_rel_c_e(1:2));
                Vv = abs(v_rel_c_e(3));
                L_over_D = Vh / max(Vv, 1e-3);
                L_over_D = min(L_over_D, 0.3);
                L_mag_c = norm(f_c) * L_over_D;

                f_l_c = L_mag_c * lift_dir_c;

                if any(isnan(f_l_c))
                    f_l_c = [0; 0; 0];
                end
            end

            obj.lift_force_p = f_l_p;
            obj.lift_force_c = f_l_c;

            % Combined lift and drag force
            f_p = f_p + f_l_p;
            f_c = f_c + f_l_c;
        end

        function [f_p, f_c] = calc_riser_force(obj)
            % CALC_RISER_FORCE Calculates force in the riser
            %   This function calculates the force in the riser, modeled by
            %   a spring-damper linkage that pulls the payload and
            %   parachute towards one another


            % Calculate relative inertial position of riser ends
            r_attach_p_e = obj.C_EB_p' * obj.payload.P_attach_B;
            r_attach_c_e = obj.C_EB_c' * obj.parachute.P_attach_B;

            % Set up structs
            % Payload
            obj1 = struct( ...
                'V',            obj.C_EB_p' * obj.V_p, ... % Inertial velocity
                'omega',        obj.C_EB_p' * obj.w_p, ... % Inertial angular
                'P_attach_rel', r_attach_p_e, ... % Relative riser attatchment point, inertial
                'P_attach',     obj.P_p + r_attach_p_e ...  % Absolute attachment point, inertial
                );

            % Canopy
            obj2 = struct( ...
                'V',            obj.C_EB_c' * obj.V_c, ...
                'omega',        obj.C_EB_c' * obj.w_c, ...
                'P_attach_rel', r_attach_c_e, ...
                'P_attach',     obj.P_c + r_attach_c_e ...
                );

            % Spring constants for the riser
            spring = struct('l0', obj.parachute.l0, ...
                'k', obj.parachute.k_riser, ...
                'c', obj.parachute.c_riser);

            % The spring force
            F_spring_e = spring_force(obj1, obj2, spring, true);

            % Rotate force into body frame. F_c = -F_p
            f_p = obj.C_EB_p   *  F_spring_e;
            f_c = obj.C_EB_c   * -F_spring_e;
        end
    end
end
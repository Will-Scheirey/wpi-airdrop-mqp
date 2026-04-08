classdef Parachute_Model_Wind < Parachute_Model_Simple
    %PARACHUTE_MODEL_SIMPLE Airdrop model accounting for winds
    %   This class extends the original parachute model to incoroporate
    %   effects from windspeed and direction at varying altitudes.

    properties
        % WIND_DATA Table of Windspeed and direction at varying altitudes
        wind_data
    end

    methods
        function obj = Parachute_Model_Wind(payload, parachute, x0, wind_data)
            % PARACHUTE_MODEL_WIND Construct an instance of this class
            %
            % INPUTS:
            %   payload   : The Payload object for the simulation
            %   parachute : The Parachute object for the simulation
            %   x0        : The initial state vector for the system
            %   wind_data : Table of windspeed and direction
            %       .alt_agl        [ m   ]
            %       .win_speed      [ m/s ]
            %       .wind_direction [ deg ] from North
            %
            % OUTPUTS:
            %   obj : The new Parachute_Model_Wind object

            obj = obj@Parachute_Model_Simple(payload, parachute, x0);
            obj.wind_data = wind_data;
        end

        function wind_vec = get_wind(obj, alt)
            % GET_WIND Returns the wind vector at a given altitude
            % 
            % INPUTS:
            %   obj : The Parachute_Model_Wind object
            %   alt : The altitude in meters
            % 
            % OUTPUTS:
            %   wind_vec : The wind vector [ m/s ] ENU

            % Interpolate the windspeed and direction from the data
            wind_speed_interp = interp1(obj.wind_data.alt_agl, obj.wind_data.win_speed,      alt);
            wind_angle_interp = interp1(obj.wind_data.alt_agl, obj.wind_data.wind_direction, alt);
            
            % Create the wind vector. Negate the value to correct direction
            wind_vec = -wind_speed_interp .* [sind(wind_angle_interp); cosd(wind_angle_interp); 0];
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

            % Get wind
            alt = obj.P_p(3);
            wind_e = obj.get_wind(alt);

            % Convert velocities to inertial frame for windspeed
            Vp_e = obj.C_EB_p' * obj.V_p;
            Vc_e = obj.C_EB_c' * obj.V_c;

            % Calculate windspeed
            v_rel_p_e = Vp_e - wind_e;
            v_rel_c_e = Vc_e - wind_e;

            % Convert windspeed back into body frame
            v_rel_p_b = obj.C_EB_p * v_rel_p_e;
            v_rel_c_b = obj.C_EB_c * v_rel_c_e;

            % Get angle of attack
            aoa_p = flight_angles(v_rel_p_b, obj.C_EB_p);
            aoa_c = flight_angles(v_rel_c_b, obj.C_EB_c);

            % Calculate drag force using windspeed vectors
            f_p = -0.5 * obj.rho * obj.payload.CdS(aoa_p)   * v_rel_p_b * norm(v_rel_p_b);
            f_c = -0.5 * obj.rho * obj.parachute.CdS(aoa_c) * v_rel_c_b * norm(v_rel_c_b);

            % If windspeed is zero, normalizing will give nan
            if any(isnan(f_p)), f_p = [0;0;0]; end
            if any(isnan(f_c)), f_c = [0;0;0]; end

            obj.aoa_p_curr = aoa_p;
            obj.aoa_c_curr = aoa_c;
            
            obj.drag_force_p = f_p;
            obj.drag_force_c = f_c;
            
            % ==============================
            % --- Parachute Lift         ---
            % ==============================
            %   L = tan(aoa)*D
            % Lift direction chosen as the component opposite gravity
            % that is perpendicular to the relative wind.
            
            f_l_p = [0; 0; 0];
            f_l_c = [0; 0; 0];
            
            v_mag_c = norm(v_rel_c_b);
            if v_mag_c > 1e-6
                v_hat_c = v_rel_c_b / v_mag_c;
            
                g_b_c = obj.C_EB_c * obj.g_vec_e;
                g_up_b_c = -g_b_c / norm(g_b_c);
            
                lift_dir_c = g_up_b_c - dot(g_up_b_c, v_hat_c) * v_hat_c;
            
                if norm(lift_dir_c) < 1e-6
                    lift_dir_c = cross(v_hat_c, [0; 1; 0]);
                    if norm(lift_dir_c) < 1e-6
                        lift_dir_c = cross(v_hat_c, [1; 0; 0]);
                    end
                end
            
                lift_dir_c = lift_dir_c / norm(lift_dir_c);
            
                Vh = norm(v_rel_c_e(1:2));     % horizontal speed
                Vv = abs(v_rel_c_e(3));        % vertical speed magnitude
                L_over_D = Vh / max(Vv, 1e-3); % avoid divide-by-zero
                
                % Clamp to keep parachute realistic/stable (round canopies are small L/D)
                L_over_D = min(L_over_D, 0.3);
                
                L_mag_c = norm(f_c) * L_over_D;
                
                f_l_c = L_mag_c * lift_dir_c;
            
                if any(isnan(f_l_c))
                    f_l_c = [0; 0; 0];
                end
            end
            
            obj.lift_force_p = f_l_p;
            obj.lift_force_c = f_l_c;
            
            f_p = f_p + f_l_p;
            f_c = f_c + f_l_c;
        end
    end
end
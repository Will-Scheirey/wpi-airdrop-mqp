classdef Two_Stage_Model < Parachute_Model_Wind
    % PARACHUTE_MODEL_SIMPLE Dynamical model with two-stage parachute system
    %   This model incorporates effects of wind, along with enabling
    %   simulation of a full two-stage drop, with two different parachutes.
    %   Parachutes can be given cut time and deployment time.

    properties
        % DROGUE_PARACHUTE The drogue parachute object
        drogue_parachute
        % MAIN_PARACHUTE The main parachute object
        main_parachute
    end

    methods
        function obj = Two_Stage_Model(payload, drogue_parachute, main_parachute, x0, wind_data)
            % TWO_STAGE_MODEL Construct an instance of this class
            %
            % INPUTS:
            %   payload          : The Payload object for the simulation
            %   drogue_parachute   : The main parachute object
            %   main_parachute : The drogue parachute object
            %   x0               : The initial state vector for the system
            %   wind_data        : Table of windspeed and direction
            %       .alt_agl        [ m   ]
            %       .win_speed      [ m/s ]
            %       .wind_direction [ deg ] from North
            %
            % OUTPUTS:
            %   obj : The new Two_Stage_Model object

            obj = obj@Parachute_Model_Wind(payload, drogue_parachute, x0, wind_data);

            obj.drogue_parachute = drogue_parachute;
            obj.main_parachute = main_parachute;
        end

        function x_dot = ode_fcn(obj, t, x)
            % ODE_FCN The ODE function, called by the ODE solver
            %   This function calls the underlying ode_fcn of its
            %   superclass, but first checks which parachute should be
            %   used. The .parachute property on the superclass is set to
            %   either .drogue_parachute or .main_parachute depending on
            %   the parameters
            %
            % INPUTS:
            %   obj : The Two_Stage_Model object
            %   t   : The timestep
            %   x   : The current state vector
            %
            % OUTPUTS:
            %   x_dot : The calculated state derivative vector

            % Check if the main parachute should be deployed
            if t >= obj.main_parachute.t_deploy

                if ~obj.main_parachute.is_deployed
                    % If the main parachute hasn't already been deployed,
                    % set the parachute's current position and velocity to
                    % that of the payload's, since the main parachute is
                    % attached to the payload before deployment
                    x(14:16) = x(1:3);
                    x(17:19) = x(4:6);
                end
                obj.main_parachute.is_deployed = true;
                obj.parachute = obj.main_parachute;

                % Check if the drogue parachute should be cut
            elseif t >= obj.drogue_parachute.t_cut
                % If the drogue parachute is cut, set is_deployed to false,
                % meaning it will have no area

                obj.drogue_parachute.is_deployed = false;
                obj.parachute = obj.drogue_parachute;

            elseif t >= obj.drogue_parachute.t_deploy
                obj.drogue_parachute.is_deployed = true;

                obj.parachute = obj.drogue_parachute;
            end
            
            % Call the underlying ode function
            x_dot = ode_fcn@Parachute_Model_Wind(obj, t, x);
        end
    end
end

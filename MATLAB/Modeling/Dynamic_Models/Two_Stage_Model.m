classdef Two_Stage_Model < Parachute_Model_Wind
    %PARACHUTE_MODEL_SIMPLE Summary of this class goes here
    %   Detailed explanation goes here

    properties
        parachute1
        parachute2
    end

    methods
        function obj = Two_Stage_Model(payload, parachute1, parachute2, x0, wind_data)
            %PARACHUTE_MODEL_SIMPLE Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@Parachute_Model_Wind(payload, parachute1, x0, wind_data);

            obj.parachute = parachute1;
            obj.parachute1 = parachute1;
            obj.parachute2 = parachute2;
            obj.payload   = payload;
            obj.wind_data = wind_data;
        end

        function x_dot = ode_fcn(obj, t, x)
            % Check if parachute should deploy
            if t >= obj.parachute2.t_deploy
                if ~obj.parachute2.is_deployed
                    fprintf('*** PARACHUTE 2 DEPLOYED at t = %.2f seconds ***\n', t);
                end
                obj.parachute2.is_deployed = true;
                obj.parachute = obj.parachute2;

            elseif t >= obj.parachute1.t_deploy
                if ~obj.parachute1.is_deployed
                    fprintf('*** PARACHUTE 1 DEPLOYED at t = %.2f seconds ***\n', t);
                end
                obj.parachute1.is_deployed = true;
                obj.parachute = obj.parachute1;
            end
                
            x_dot = ode_fcn@Parachute_Model_Wind(obj, t, x);

            % obj.parachute.R
        end
    end
end
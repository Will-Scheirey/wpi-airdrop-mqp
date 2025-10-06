classdef Parachute_Model_Wind < Parachute_Model_Simple
    %PARACHUTE_MODEL_WIND Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
    end

    methods
        function obj = Parachute_Model_Wind(payload, parachute, x0)
            %PARACHUTE_MODEL_SIMPLE Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@Parachute_Model_Simple(payload, parachute, x0);

            obj.parachute = parachute;
            obj.payload   = payload;
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end
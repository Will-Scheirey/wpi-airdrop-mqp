classdef Dynamic_Model < matlab.System
    %DYNAMIC_MODEL Summary of this class goes here
    %   Detailed explanation goes here

    properties
        x0

        g
        g_vec_e

        time_history
        state_history
    end

    methods
        function obj = Dynamic_Model(x0)
            %DYNAMIC_MODEL Construct an instance of this class
            %   Detailed explanation goes here

            obj.x0            = x0;

            obj.g            = 9.81;           % Gravitational acceleration [m  s^-2]
            obj.g_vec_e      = [0; 0; -obj.g]; % Gravity vector in ECEF     [m  s^-2]
        end

        function [t, y] = run_model(obj, x0, tspan, rel_tol, abs_tol)
            obj.x0 = x0;

            if nargin < 4
                rel_tol = 1e-12;
            end
            if nargin < 5
                abs_tol = 1e-12;
            end

            options = odeset('RelTol', rel_tol, 'AbsTol', abs_tol); % Set solver tolerance

            [t, y] = ode89(@obj.ode_fcn, tspan, x0, options);

            obj.time_history  = t;
            obj.state_history = y;
        end

        function [a_b, alpha_b] = calc_accel(~, V_b, w_b, m, I, F_b, M_b)
            a_b     = F_b/m - cross(w_b, V_b);       % Body accelerations
            alpha_b = I \ (M_b - cross(w_b, I*w_b)); % Angular accelerations
        end

    end
    methods (Abstract)
        get_states(obj, x);
        x_dot = ode_fcn(obj, t, x_curr);
    end

    methods (Access = protected)
        function y = stepImpl(obj,u)
            % Implement algorithm. Calculate y as a function of input u and
            % internal states.
            y = u;
        end
    end
end
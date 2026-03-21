classdef Dynamic_Model < handle
    % DYNAMIC_MODEL Abstract class for running a dynamic model
    %   This class also has a function for calculating translational and
    %   angular acceleration from force, moment, mass, and moment of
    %   inertia

    properties
        % X0 Initial states
        x0

        % G Gravity norm
        g

        % G_VEC_E Gravity vector in inertial frame
        g_vec_e

        % TIME_HISTORY Time series of propagator output
        time_history
        % STATE_HISTORY State history of propagator output
        state_history
    end

    methods
        function obj = Dynamic_Model(x0)
            % DYNAMIC_MODEL Creates the dynamic model class

            obj.x0            = x0;

            obj.g            = 9.81;           % Gravitational acceleration [m  s^-2]
            obj.g_vec_e      = [0; 0; -obj.g]; % Gravity vector in ECEF     [m  s^-2]
        end

        function [t, y] = run_model(obj, x0, tspan, rel_tol, abs_tol)
            % RUN_MODEL Runs the dynamic model through an ODE solver
            %   Tolerances can be supplied, but are not required. The
            %   simulation will end when the object hits the ground
            % 
            % INPUTS:
            %   obj     : The Dynamic_Model object
            %   x0      : The initial states
            %   tspan   : The tspan for propagator outputs
            %   rel_tol : (Optional) Relative tolerance for propagator
            %   abs_tol : (Optional) Absolute tolerance for propagaror
            %
            % OUTPUTS:
            %   t : The output time series from the propagator
            %   y : The output states from the propagator

            obj.x0 = x0;

            % Set defaults
            if nargin < 4, rel_tol = 1e-6; end
            if nargin < 5, abs_tol = 1e-8; end

            % Setup the ODE solver with event checking for hitting the
            % ground
            options = odeset('RelTol', rel_tol, 'AbsTol', abs_tol, 'Events', @myEvent,'MaxStep',0.2); % Set solver tolerance

            % Run the ODE solver using the ODE function in obj
            [t, y] = ode15s(@obj.ode_fcn, tspan, x0, options);
            obj.time_history  = t;
            obj.state_history = y;

            % Event checking
            function [value, isterminal, direction] = myEvent(~, y)
                value = y(3) > 0;
                isterminal = 1;     % Stop the integration
                direction = -1;     % descending through 0
            end
        end

        function [a_b, alpha_b] = calc_accel(~, V_b, w_b, m, I, F_b, M_b)
            % CALC_ACCEL Calculates accelerations from forces and moments
            %   This function is for a basic rigidbody model
            %
            % INPUTS:
            %   V_b : Velocity in the body frame
            %   w_b : Angular velocity in the body frame
            %   m   : Mass of the body
            %   I   : Moment of inertia of the body
            %   F_b : Resultant force on the object in the body frame
            %   M_b : Resultant moment on the object in the body frame
            %
            % OUTPUTS:
            %   a_b     : Resulting translational acceleration in the body frame
            %   alpha_b : Resulting angular acceleration in the body frame

            a_b     = F_b/m - cross(w_b, V_b);       % Body accelerations
            alpha_b = I \ (M_b - cross(w_b, I*w_b)); % Angular accelerations
        end

    end

    % Abstract methods to be implemented by subclasses
    methods (Abstract)
        get_states(obj, x); % For internal use
        x_dot = ode_fcn(obj, t, x_curr); % Function for the ODE solver
    end
end
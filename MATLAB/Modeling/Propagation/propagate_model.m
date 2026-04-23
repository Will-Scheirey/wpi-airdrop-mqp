function [t, y, model_obj] = propagate_model(NameValueArgs)
    % PROPAGATE_MODEL Propagates a dynamic model
    % 
    % INPUTS:
    %   NameValueArgs : Parameters for propagation
    %
    % OUTPUTS:
    %   t         : The ode solver output time series
    %   y         : The ode solver output states
    %   model_obj : The dynamical model object used for propagation
    arguments
        % USE_DRAG Whether to use drag in the simulation
        NameValueArgs.use_drag
        % PAYLOAD The payload object
        NameValueArgs.payload
        % PARACHUTE The parachute, or the drogue parachute for 2-stage
        NameValueArgs.parachute
        % PARACHUTE2 The main parachute for 2-stage
        NameValueArgs.parachute2
        % X0 Initial states
        NameValueArgs.x0
        % TSPAN Desired output time series
        NameValueArgs.tspan
        % MODEL A reference to the class of the dynamical model to use
        NameValueArgs.model
        % RISER: Whether to incorporate the riser force
        NameValueArgs.riser
        % VARIABLE_PARACHUTE_MASS : Whether to use variable parachute mass
        NameValueArgs.variable_parachute_mass
        % DAMPING : Whether to include damping in the riser
        NameValueArgs.damping
        % WEATHER : Weather for the drop
        NameValueArgs.weather
    end

    if isfield(NameValueArgs, 'use_drag')
        use_drag = NameValueArgs.use_drag;
    else
        use_drag = true;
    end

    % ========================
    % --- Physical Objects ---
    % ========================

    if isfield(NameValueArgs, 'payload')
        payload = NameValueArgs.payload;
    else
        % --- Default Payload Properties ---
        a22_width  = in2m(48);    % [m]
        a22_length = in2m(83);    % [m]
        a22_height = in2m(43);    % [m]
        a22_mass   = lb2kg(2200); % [kg]
        
        payload = Box(a22_width, ...
            a22_length, ...
            a22_height, ...
            a22_mass, ...
            use_drag);
    end

    if isfield(NameValueArgs, 'variable_parachute_mass')
        variable_parachute_mass = NameValueArgs.variable_parachute_mass;
    else
        variable_parachute_mass = true;
    end

    if isfield(NameValueArgs, 'parachute')
        parachute = NameValueArgs.parachute;
    else
         % --- Default Parachute Properties ---
        canopy_radius = in2m(50);       % [m]
        canopy_mass = 2;         % [kg]
        
        riser_length = 10;       % [m] (resting riser length)

        if isfield(NameValueArgs, 'riser')
            if NameValueArgs.riser
                riser_k = 10000;         % [N/m]       Riser stiffness
                riser_c = 1000;          % [kg s^-1]   Riser damping coefficient
            else
                riser_k = 0;
                riser_c = 0;
            end
        else
            riser_k = 10000;         % [N/m]       Riser stiffness
            riser_c = 1000;          % [kg s^-1]   Riser damping coefficient
        end

        if isfield(NameValueArgs, 'damping')
            if ~NameValueArgs.damping
                riser_c = 0;
            end
        end
        
        canopy_efficiency = 1;   % []
        canopy_porosity =   0.2; % []
        
        parachute = Parachute(canopy_radius, ...
            canopy_mass, ...
            riser_length, ...
            riser_k, ...
            riser_c, ...
            canopy_efficiency, ...
            canopy_porosity, ...
            use_drag, ...
            variable_parachute_mass);
    end

    % ==========================
    % --- Initial Conditions ---
    % ==========================

    if isfield(NameValueArgs, 'x0')
        x0 = NameValueArgs.x0;
    else
        v0 = 300;
        % --- Payload ---
        P0   = [0; 0; 3000];              % ENU position      [m]
        V_p0 = [v0; 0; 0];                % ENU velocity      [m   s^-1]
        e_p0 = eul2quat([0, 0, 0])'; % Orientation
        w_p0 = [0; 0; 0];                % Body angular rates [rad s^-1]
        
        % --- Parachute ---
        P0_c = P0 + [0; 0; parachute.l0];           % ENU Position      [m]
        V_c0 = [v0; 0; 0];                % ENU velocity      [m   s^-1]
        e_c0 = eul2quat([0, 0, 0])';  % Orientation
        w_c0 = [0; 0; 0];               % Body angular rates [rad s^-1]
        
        x0   = [
            P0;
            body2enu_rotm(e_p0) * V_p0;
            e_p0;
            w_p0;
        
            P0_c
            body2enu_rotm(e_c0) * V_c0;
            e_c0;
            w_c0;
            ];
    end

    if isfield(NameValueArgs, 'tspan')
        tspan = NameValueArgs.tspan;
    else
        tspan = linspace(0, 90, 30000);
    end

    if isfield(NameValueArgs, 'model')
        model = NameValueArgs.model;
    else
        model = @Parachute_Model_Simple;
    end

    % If 'parachute2' has been set, we know the drop is for two-stage, so
    % call to two-stage constructor. Otherwise if weather has been set, we
    % know it is the wind model. In the future, it would be good to adjust
    % this so the two-stage model can be run without wind, which it
    % currently can't
    
    if isfield(NameValueArgs, 'parachute2')
        model_obj = model(payload, parachute, NameValueArgs.parachute2, x0, NameValueArgs.weather);
    elseif isfield(NameValueArgs, 'weather')
        model_obj = model(payload, parachute, NameValueArgs.weather);
    else
        model_obj = model(payload, parachute, x0);
    end

    % Run the model
    [t, y] = model_obj.run_model(x0, tspan);
end
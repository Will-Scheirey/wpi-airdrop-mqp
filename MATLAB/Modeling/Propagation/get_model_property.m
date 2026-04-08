function [varargout] = get_model_property(t, y, model, varargin)
    % GET_MODEL_PROPERTY Gets properties of a simulation for each timestep
    %   This is a utility function that allows the extraction of
    %   internally-calculated properties like forces or moments that are
    %   otherwise not available in the state outputs from the ode solver
    %
    %   The ode function is called for each of the timesteps to set the
    %   properties, and properties are retrieved by their field name in the 
    %   dynamical model class
    % 
    %   
    %   This function will throw an error of a nonexistent field name is
    %   passed
    % 
    % INPUTS:
    %   t     : The time series for extraction
    %   y     : The states for each timestep in the series
    %   model : The dynamical model object
    %   varargin : The class field names of the desired properties
    %
    % OUTPUTS:
    %   varagout : Property values for each field name for each timestep
    %

    num_steps = numel(t);
    num_properties = numel(varargin);

    % Create the varargout cell array and check the field names exist
    for n = 1:num_properties
        property_name = varargin{n};
        if ~isprop(model, property_name)
            error("The property name %s does not exist for the class %s", property_name, class(model));
        end
            
        property = model.(property_name);
        varargout{n} = repmat(property, 1, num_steps)' * 0;
    end

    % Call the ode function for each timestep and record the properties
    for i = 1:num_steps
        model.ode_fcn(t, y(i, :)');

        for n = 1:num_properties
            property_name = varargin{n};
            varargout{n}(i, :) = model.(property_name);
        end
    end
end

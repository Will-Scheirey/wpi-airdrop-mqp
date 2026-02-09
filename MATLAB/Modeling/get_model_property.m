%{
function property_hist = get_model_property(t, y, model, property_name)
    num_steps = numel(t);

    if ~isprop(model, property_name)
        error("The property name %s does not exist for the class %s", property_name, class(model));
    end

    for i = 1:num_steps
        model.ode_fcn(t, y(i, :)');
        property_hist(i, :) = get(model, property_name);
    end
end
%}

function [varargout] = get_model_property(t, y, model, varargin)
    num_steps = numel(t);

    num_properties = numel(varargin);

    for n = 1:num_properties
        property_name = varargin{n};
        if ~isprop(model, property_name)
            error("The property name %s does not exist for the class %s", property_name, class(model));
        end
            
        property = get(model, property_name);
        varargout{n} = repmat(property, 1, num_steps)' * 0;
    end

    for i = 1:num_steps
        model.ode_fcn(t(i), y(i, :)');

        for n = 1:num_properties
            property_name = varargin{n};
            varargout{n}(i, :) = get(model, property_name);
        end
    end
end

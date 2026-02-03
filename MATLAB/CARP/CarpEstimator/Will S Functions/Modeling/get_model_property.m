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
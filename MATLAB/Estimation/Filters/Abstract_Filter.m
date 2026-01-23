classdef Abstract_Filter < handle
    properties
        x_inds

        R
        Q
    end

    methods (Abstract)
        [innovation, K, S] = update(obj, y)
        predict(obj, u)

        dfdx = f_jacobian_states(obj, u);
        dhdx = h_jacobian_states(obj);
        y    = h(obj);
        dxdt = f(obj);
    end
end
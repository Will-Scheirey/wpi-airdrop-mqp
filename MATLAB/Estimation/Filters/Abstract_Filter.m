classdef Abstract_Filter < handle
    % ABSTRACT_FILTER An abstracted filter class
    %   This class just defines the most basic properties of a filter,
    %   which are the same across both Kalman/information filters and
    %   forward-backward smoothers
    
    properties
        % X_INDS Struct of indices corresponding with state names
        %   Example: x_inds = struct('position', 1:3, 'altitude', 4)
        x_inds

        % R Measurement covariance matrix
        R
        % Q Process noise matrix
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
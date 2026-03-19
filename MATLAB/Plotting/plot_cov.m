function p = plot_cov(err, cov, t)
% PLOT_COV Plots error and variance bounds
%
% INPUTS:
%   err : The error (or covariance, if nargin == 1)
%   cov : The covariance
%   t   : (Optional) The time series for plotting

if nargin < 3
    t = 1:length(err);
end

if nargin == 1
    plot_cov_(t, squeeze(err));
    return
end

% Reduce dimensions
cov = squeeze(cov);

% Plot error
p = plot(t, err, '-r', 'MarkerSize', 1.5, 'DisplayName', 'Error'); hold on
plot_cov_(t, cov);

    function plot_cov_(t, cov)
        % Helper function for plotting +/- variance bounds
        plot(t, sqrt(cov), '-b', 'LineWidth', 1, 'DisplayName', 'Covariance'); hold on
        plot(t, -sqrt(cov), '-b', 'LineWidth', 1, 'HandleVisibility', 'off');
    end

end
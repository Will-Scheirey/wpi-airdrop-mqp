function p = plot_cov(err, cov, t)
    cov = squeeze(cov);
    if nargin < 3
        t = 1:length(err);
    end
    p = plot(t, err, '-r', 'MarkerSize', 1.5, 'DisplayName', 'Error'); hold on
    plot(t, sqrt(cov), '-b', 'LineWidth', 1, 'DisplayName', 'Covariance');
    plot(t, -sqrt(cov), '-b', 'LineWidth', 1, 'HandleVisibility', 'off');
end
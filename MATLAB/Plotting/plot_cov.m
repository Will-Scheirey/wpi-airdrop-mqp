function p = plot_cov(err, cov, t)

if nargin < 3
    t = 1:length(err);
end

if nargin == 1
    plot_cov_(t, squeeze(err));
    return
end

cov = squeeze(cov);

p = plot(t, err, '-r', 'MarkerSize', 1.5, 'DisplayName', 'Error'); hold on
plot_cov_(t, cov);

    function plot_cov_(t, cov)
        plot(t, sqrt(cov), '-b', 'LineWidth', 1, 'DisplayName', 'Covariance'); hold on
        plot(t, -sqrt(cov), '-b', 'LineWidth', 1, 'HandleVisibility', 'off');
    end

end
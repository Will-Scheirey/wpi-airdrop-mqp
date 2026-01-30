function plot_cov(variance)
    variance = squeeze(variance);
    plot(sqrt(variance), '.r', 'DisplayName', 'Covariance', 'LineWidth', 1.5); hold on;
    plot(-sqrt(variance), '.r', 'HandleVisibility', 'off', 'LineWidth', 1.5);
end
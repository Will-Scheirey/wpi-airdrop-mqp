function fig_idx = new_fig(fig_idx)
    % NEW_FIG Creates a new figure with a specified index and returns new
    %
    % INPUTS:
    %   fig_idx : The number for the new figure
    %
    % OUTPUTS:
    %   fig_idx : The incremented figure index
    figure(fig_idx); fig_idx = fig_idx + 1;
end
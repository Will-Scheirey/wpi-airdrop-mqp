function fig_idx = plot_estimates(data_in, fig_idx)

if nargin < 2
    fig_idx = 1;
end

t_plot = data_in.t_plot;
t_plot_drop = data_in.t_plot_drop;
tspan = data_in.tspan;

p_est = data_in.estimates.pos;
v_est = data_in.estimates.vel;
e_est = data_in.estimates.quat;
w_b_est = data_in.estimates.gyro_bias;
a_b_est = data_in.estimates.accel_bias;
p_b_est = data_in.estimates.pos_bias;
m_b_est = data_in.estimates.mag_bias;
b_b_est = data_in.estimates.baro_bias;

data_gps = data_in.measurements.gps;
acc_gps = data_in.measurements.acc_gps;
data_gps_vel = data_in.measurements.gps_vel;

kf = data_in.kf;

drop_time = data_in.drop_info.time_drop;

%% Plot Values
fig_idx = new_fig(fig_idx);
clf
plot(t_plot, p_est, 'LineWidth', 2)
legend("P_0^E", "P_1^E", "P_2^E")
xlabel("Time (s)")
ylabel("Position (m)")
title("Position vs. Time")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
subplot(3,1,1)
plot(data_gps.time, data_gps.data(:, 1), '.', 'MarkerSize', 10); hold on
plot(t_plot, p_est(:,1), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_1^E vs. Time")
xlim(t_plot_drop)

subplot(3,1,2)
plot(data_gps.time, data_gps.data(:, 2), '.', 'MarkerSize', 10); hold on
plot(t_plot, p_est(:,2), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_2^E vs. Time")
xlim(t_plot_drop)

subplot(3,1,3)
plot(data_gps.time, data_gps.data(:, 3), '.', 'MarkerSize', 10); hold on
plot(t_plot, p_est(:,3), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Position (m)")
title("P_3^E vs. Time")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
plot(data_gps.time, acc_gps(:, 1), 'DisplayName', 'Horizontal', 'LineWidth', 2); hold on
plot(data_gps.time, acc_gps(:, 2), 'DisplayName', 'Vertical', 'LineWidth', 2);
xlim(t_plot_drop)
legend
title("GPS Accuracy")
xlabel("Time (s)")
ylabel("Accuracy (m)")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, v_est, 'LineWidth', 2); hold on
plot(t_plot, vecnorm(v_est, 2, 2), 'LineWidth', 1.5)
legend("V_0^E", "V_1^E", "V_2^E", "Velocity Norm")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("ECEF Velocity vs. Time")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
subplot(3,1,1)
plot(data_gps_vel.time, data_gps_vel.data(:, 1), '.', 'MarkerSize', 10); hold on
plot(t_plot, v_est(:,1), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("V_1^E vs. Time")
xlim(t_plot_drop)

subplot(3,1,2)
plot(data_gps_vel.time, data_gps_vel.data(:, 2), '.', 'MarkerSize', 10); hold on
plot(t_plot, v_est(:,2), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
title("V_2^E vs. Time")
xlim(t_plot_drop)

subplot(3,1,3)
plot(data_gps_vel.time, data_gps_vel.data(:, 3), '.', 'MarkerSize', 10); hold on
plot(t_plot, v_est(:,3), 'LineWidth', 2); hold on;
legend("Measurement", "Estimate")
xlabel("Time (s)")
ylabel("Velocity (m)")
title("V_3^E vs. Time")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, e_est, 'LineWidth', 2)
legend("e0", "e1", "e2", "e3")
xlabel("Time (s)")
ylabel("Value")
title("Quaternion vs. Time")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, m_b_est(:, 1), 'DisplayName', 'X', 'LineWidth', 1.5); hold on
plot(t_plot, m_b_est(:, 2), 'DisplayName', 'Y', 'LineWidth', 1.5);
plot(t_plot, m_b_est(:, 3), 'DisplayName', 'Z', 'LineWidth', 1.5);
xlabel("Time (s)")
ylabel("Bias Estimate (gauss)")
title("Magnetometer Bias Estimate vs Time")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
plot(tspan, kf.accel_calc_all, 'LineWidth', 2); hold on
legend("a_1", "a_2", "a_3")
xlim([t_plot(1)+1, t_plot(end)])
% xlim(t_plot_drop)
title("Estimated Acceleration Components")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, w_b_est, 'LineWidth', 1.5); hold on
legend("b_1", "b_2", "b_3")
xlabel("Time (s)")
ylabel("Bias (rad/s)")
xlim([t_plot(1)+1, t_plot(end)])

title("Gyro Bias Estimates")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, a_b_est, 'LineWidth', 1.5); hold on
legend("b_1", "b_2", "b_3")
xlabel("Time (s)")
ylabel("Bias (m/s^2)")
xlim([t_plot(1)+1, t_plot(end)])

title("Acceleration Bias Estimates")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, b_b_est, 'LineWidth', 1.5); hold on
xlabel("Time (s)")
ylabel("Bias (m)")
xlim([t_plot(1)+1, t_plot(end)])

title("Barometer Bias Estimates")

fig_idx = new_fig(fig_idx);
clf
plot(t_plot, p_b_est, 'LineWidth', 1.5); hold on
legend("b_1", "b_2", "b_3")
xlabel("Time (s)")
ylabel("Bias (m)")
xlim([t_plot(1)+1, t_plot(end)])

title("GPS Bias Estimates")

%% Plot Innovation

fig_idx = new_fig(fig_idx);
clf
plot(tspan, kf.inno_hist(kf.measurement_ranges{2}, :), '.-', 'MarkerSize', 10);
legend("m_x", "m_y", "m_z")
title("Magnetometer Innovation")
xlabel("Time (s)")
ylabel("Mag Innovation")
xlim(t_plot_drop)

fig_idx = new_fig(fig_idx);
clf
pos_inno = kf.inno_hist(1:3, :)';
good_pos_inno = ~isnan(pos_inno(:, 1));
pos_inno_tstep = 1:length(pos_inno);

x_pos_inno = pos_inno(good_pos_inno, 1);

idx = 1;
plot_cov(kf.P_hist(idx,idx,:)); hold on
plot(pos_inno_tstep(good_pos_inno), x_pos_inno)
xlabel("Timestep")
ylabel("Variance (rm/s)")
title("X Position Variance and Innovation")
% xlim([0, 26313])
ylim([-7, 7])
legend("Variance", "Innovation")

fig_idx = new_fig(fig_idx);
clf
y_pos_inno = pos_inno(good_pos_inno, 2);
idx = 2;
plot_cov(kf.P_hist(idx,idx,:)); hold on
plot(pos_inno_tstep(good_pos_inno), y_pos_inno)
xlabel("Timestep")
ylabel("Variance (rm/s)")
title("X_1 Position Variance")

fig_idx = new_fig(fig_idx);
clf

idx = 7;
plot_cov(kf.P_hist(idx,idx,:)); hold on

e_inno = kf.inno_hist(4:6, :)';
good_e_inno = ~isnan(e_inno(:, 1));
e_inno_tstep = 1:length(e_inno);

e_0_inno = e_inno(good_e_inno, 1);

plot(e_inno_tstep(good_e_inno), e_0_inno)

xlabel("Timestep")
ylabel("Variance ")
title("e_0 Quaternion Part Variance")
% xlim([0, 26313])
ylim([-1,1]*0.15)
legend("Variance", "Innovation")

fig_idx = new_fig(fig_idx);
clf
idx = 8;
plot_cov(kf.P_hist(idx,idx,:)); hold on
e_inno = kf.inno_hist(4:6, :)';
good_e_inno = ~isnan(e_inno(:, 1));
e_inno_tstep = 1:length(e_inno);

e_1_inno = e_inno(good_e_inno, 2);

plot(e_inno_tstep(good_e_inno), e_1_inno)

xlabel("Timestep")
ylabel("Variance ")
title("e_1 Quaternion Part Variance")
ylim([-1,1]*0.15)
legend("Variance", "Innovation")

fig_idx = new_fig(fig_idx);
clf
plot3(p_est(:,1), p_est(:,2), p_est(:,3), '.-', 'MarkerSize', 10, 'DisplayName', 'Estimate', 'LineWidth', 0.5); hold on;
plot3(p_est(1,1), p_est(1,2), p_est(1,3), '.r', 'MarkerSize', 30, 'DisplayName', 'Start', 'LineWidth', 1); hold on;
plot3(p_est(end,1), p_est(end,2), p_est(end,3), '.g', 'MarkerSize', 30, 'DisplayName', 'End', 'LineWidth', 1); hold on;
legend
axis equal
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
title("Trajectory")

fig_idx = new_fig(fig_idx);
clf
plot3(p_est(:,1), p_est(:,2), p_est(:,3), '.-', 'MarkerSize', 10, 'DisplayName', 'Estimate', 'LineWidth', 0.5); hold on;
plot3(p_est(1,1), p_est(1,2), p_est(1,3), '.r', 'MarkerSize', 30, 'DisplayName', 'Start', 'LineWidth', 1); hold on;
plot3(p_est(end,1), p_est(end,2), p_est(end,3), '.g', 'MarkerSize', 30, 'DisplayName', 'End', 'LineWidth', 1); hold on;
legend
axis equal
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
title("Trajectory")

fig_idx = new_fig(fig_idx);
clf

drop_idx = t_plot > drop_time;
x_pos = p_est(drop_idx, 1) - p_est(find(drop_idx, 1), 1);
y_pos = p_est(drop_idx, 2) - p_est(find(drop_idx, 1), 2);
z_pos = p_est(drop_idx, 3);

yyaxis left
plot(t_plot(drop_idx) - t_plot(find(drop_idx, 1)), x_pos, 'LineWidth', 1.5, 'DisplayName', 'X'); hold on;
plot(t_plot(drop_idx) - t_plot(find(drop_idx, 1)), y_pos, 'LineWidth', 1.5, 'DisplayName', 'Y'); hold on;
ylabel("X, Y Position (m)")

yyaxis right
plot(t_plot(drop_idx) - t_plot(find(drop_idx, 1)), z_pos, 'LineWidth', 1.5, 'DisplayName', 'z'); hold on;
ylabel("Altitude (m)")

xlabel("Time (s)")
legend
title("Position vs. Time")

fig_idx = new_fig(fig_idx);
clf

p_est = p_est(t_plot > t_plot_drop(1), :);
p_est = p_est - p_est(end, :);

plot3(p_est(:,1), p_est(:,2), p_est(:,3), '.-', 'MarkerSize', 10, 'DisplayName', 'Estimate', 'LineWidth', 0.5); hold on;
plot3(p_est(1,1), p_est(1,2), p_est(1,3), '.r', 'MarkerSize', 30, 'DisplayName', 'Start', 'LineWidth', 1); hold on;
plot3(p_est(end,1), p_est(end,2), p_est(end,3), '.g', 'MarkerSize', 30, 'DisplayName', 'End', 'LineWidth', 1); hold on;
legend
axis equal
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")
title("Trajectory")

end
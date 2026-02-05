clear; clc; close all;

t_start = 9.0e5;
t_end   = 1.01e6;

data_ASM = filter_imu_data("ASM330.csv", t_start, t_end);
data_ICM = filter_imu_data("ICM20948.csv", t_start, t_end);
data_baro = get_baro_data("LPS22.csv", t_start, t_end, 34);
data_ina  = load_INA_data(t_start, t_end);

[ax_ASM, ay_ASM, az_ASM, a_ASM] = get_imu_accel(data_ASM);
[ax_ICM, ay_ICM, az_ICM, a_ICM] = get_imu_accel(data_ICM);


%% PLOT

% Accel Components

figure(1)
clf
subplot(2, 1, 1)
plot_imu_accel(data_ASM)
title("ASM")
ylabel("Acceleration (G)")

subplot(2, 1, 2)
plot_imu_accel(data_ICM)
title("ICM")
ylabel("Acceleration (G)")
xlabel("Time (s)")

sgtitle("IMU Acceleration Readings")

figure(2)
clf

subplot(3,1,1);
plot(data_ICM.time / 1e3, ax_ICM, 'DisplayName', 'ICM', 'LineWidth', 1, 'Color', [0, 0, 1, 1]);  hold on;
plot(data_ASM.time / 1e3, ax_ASM, 'DisplayName', 'ASM', 'LineWidth', 2, 'Color', [1, 0, 0, 0.4]);
legend
xlim([data_ICM.time(1), data_ICM.time(end)]/1e3)
title("Accel X")
ylabel("Acceleration (G)")

subplot(3,1,2);
plot(data_ICM.time / 1e3, ay_ICM, 'DisplayName', 'ICM', 'LineWidth', 1, 'Color', [0, 0, 1, 1]);  hold on;
plot(data_ASM.time / 1e3, ay_ASM, 'DisplayName', 'ASM', 'LineWidth', 2, 'Color', [1, 0, 0, 0.4]);
legend
xlim([data_ICM.time(1), data_ICM.time(end)]/1e3)
title("Accel Y")
ylabel("Acceleration (G)")

subplot(3,1,3);
plot(data_ICM.time / 1e3, az_ICM, 'DisplayName', 'ICM', 'LineWidth', 1, 'Color', [0, 0, 1, 1]);  hold on;
plot(data_ASM.time / 1e3, az_ASM, 'DisplayName', 'ASM', 'LineWidth', 2, 'Color', [1, 0, 0, 0.4]);
legend
xlim([data_ICM.time(1), data_ICM.time(end)]/1e3)
title("Accel Z")
ylabel("Acceleration (G)")
xlabel("Time (s)")

sgtitle("IMU Acceleration Readings")

figure(3)
clf
plot(data_ICM.time / 1e3, a_ICM, 'DisplayName', 'ICM', 'LineWidth', 1, 'Color', [0, 0, 1, 1]);  hold on;
plot(data_ASM.time / 1e3, a_ASM, 'DisplayName', 'ASM', 'LineWidth', 2, 'Color', [1, 0, 0, 0.4]);
ylabel("Acceleration (G)")
xlabel("Time (s)")

legend

%% BARO

figure(4)

yyaxis left
plot(data_baro.time / 1000, data_baro.msl, 'DisplayName', 'Measured Altitude', 'LineWidth', 1.5)

yyaxis right
plot(data_ina.time / 1000, data_ina.current, 'DisplayName', 'Measured Servo Current', 'LineWidth', 1.5)

xlabel("Time (s)")
ylabel("Altitude (m)")
title("Measured Altitude Vs. Time")
xticks([0, 4.6, 8.6, 20, 40, 60, 80, 100])
legend

figure(5)
yyaxis right
plot(data_ASM.time / 1000, a_ASM, 'DisplayName', 'Measured Acceleration Norm', 'LineWidth', 1.5);
ylim([-1, 10])
ylabel("Acceleration (G)")

yyaxis left
plot(data_ina.time / 1000, data_ina.current, 'DisplayName', 'Measured Servo Current', 'LineWidth', 0.7)
ylabel("Current (mA)")

legend
xlabel("Time (s)")

%% For Presentation

figure(6)
clf
apogee = max(data_baro.msl);
apogee_time = data_baro.time(find(data_baro.msl == apogee));

plot(data_baro.time / 1000, data_baro.msl * 3.281, 'DisplayName', 'Measured Altitude', 'LineWidth', 1.5); hold on

plot(apogee_time / 1000, apogee * 3.281, '.r', 'MarkerSize', 30)
text(apogee_time / 1000, apogee * 3.281 + 100, sprintf("Apogee (%0.2f ft)", apogee * 3.281), "HorizontalAlignment", "center")

plot(39.629, 1428.75, '.r', 'MarkerSize', 30)
text(41, 1500, 'Main Parachute')

plot(100, 125, '.r', 'MarkerSize', 30)
text(100, 240, 'Landing', 'HorizontalAlignment', 'center')

xlabel("Time (s)")
ylabel("Altitude (ft)")
title("Measured Altitude Vs. Time")
ylim([0, 3700])

figure(7)
clf
plot(data_ina.time / 1000, data_ina.current, 'DisplayName', 'Measured Servo Current', 'LineWidth', 0.7); hold on

plot(4.271, 178.4, '.r', 'MarkerSize', 30)
text(4, 220, sprintf('Airbrakes\nDeploy'), 'HorizontalAlignment','center')

plot(8.753, 159.1, '.r', 'MarkerSize', 30)
text(9, 120, sprintf('Airbrakes\nRetract'), 'HorizontalAlignment','center')

% legend
xlabel("Time (s)")
ylabel("Current (mA)")

figure(8)
clf


function [ax, ay, az, a] = get_imu_accel(data_imu)
    ax = data_imu.accelX;
    ay = data_imu.accelY;
    az = data_imu.accelZ;

    a = vecnorm([ax, ay, az], 2, 2);
end

function plot_imu_accel(data_imu)

[ax, ay, az, a] = get_imu_accel(data_imu);

t = data_imu.time;

plot(t / 1e3, ax, 'DisplayName', 'x', 'LineWidth', 1.5); hold on
plot(t / 1e3, ay, 'DisplayName', 'y', 'LineWidth', 1.5); hold on
plot(t / 1e3, az, 'DisplayName', 'z', 'LineWidth', 1.5); hold on
legend
title("Acceleration vs Time")
xlim([t(1), t(end)]/1e3)
end

function data = filter_imu_data(filename, t_start, t_end)
data_imu = readtable(filename);
to_remove = data_imu.accelZ == -17;
data_imu = data_imu(~to_remove, :);

idx_start = find(data_imu.time > t_start, 1);
idx_end = find(data_imu.time > t_end, 1);

data = data_imu(idx_start:idx_end, :);

data.time = data.time - data.time(1);
end

function [data_baro, ground_avg] = get_baro_data(filename, t_start, t_end, zero_actual)
    data_baro = readtable(filename);

    idx_start = find(data_baro.time > t_start, 1);
    idx_end = find(data_baro.time > t_end, 1);

    msl = atmospalt(data_baro.pressure * 100);

    ground_avg = mean(msl(1:idx_start));

    data_baro = data_baro(idx_start:idx_end, :);
    data_baro.time = data_baro.time - data_baro.time(1);
    
    zero_offset = ground_avg - zero_actual;

    data_baro.msl = msl(idx_start:idx_end) - zero_offset;
end

function data_INA = load_INA_data(t_start, t_end)
    data = readtable("INA219.csv");

    data_INA = table(data{:, 1}, data{:, 2}, 'VariableNames', {'time', 'current'});

    idx_start = find(data_INA.time > t_start, 1);
    idx_end = find(data_INA.time > t_end, 1);
    
    data_INA = data_INA(idx_start:idx_end, :);
    
    data_INA.time = data_INA.time - data_INA.time(1);
end
clear; clc; close all

x0 = [
    0;
    0;
    0;

    0;
    0;
    0;

    1;
    0;
    0;
    0;

    1.1;
    1.3;
    0.2;
];

tspan = linspace(0, 10, 1000);

[t, x] = ode45(@basic_dynamic_model, tspan, x0);

subplot(3,1,1)
plot(t, x(:, 11), 'LineWidth', 1.5)
xlabel("Time (s)")
ylabel("Position (m)")
title('P_0^E')

subplot(3,1,2)
plot(t, x(:, 12), 'LineWidth', 1.5)
xlabel("Time (s)")
ylabel("Position (m)")
title('P_1^E')

subplot(3,1,3)
plot(t, x(:, 13), 'LineWidth', 1.5)
xlabel("Time (s)")
ylabel("Position (m)")
title('P_2^E')

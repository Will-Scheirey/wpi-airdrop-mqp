clear; clc; close all;
addpath("Parachute_Utils/", "Objects/", "Kinematics/", "Dynamic_Models/")

% ========================
% --- Physical Objects ---
% ========================

payload = SphereObject(1000, 0.5);
parachute = Parachute_Rigid_Hemi(7, 2, 1.225, 3);

% --- Parachute ---


% ==========================
% --- Initial Conditions ---
% ==========================

P0   = [0; 0; 10000];    % ECEF Position      [m]
V_b0 = [10; 0; 0];       % Body velocities    [m   s^-1]
% eul_b0 = [0; 0; 0];
e_b0 = eul2quat([0, pi, 0])';
w_b0 = [0; 0; 0];     % Body angular rates [rad s^-1]


P0_c = P0 + [-5; 0; 0];
V_c0 = [-100; 0; 0];    % Canopy ECEF body velocity
% eul_c0 = [0; 0; 0];
e_c0 = [1; 0; 0; 0];
w_c0 = [3; 0; 0];

x0   = [
    P0 + [0; 0; 0];
    V_b0;

    % eul_b0;
    e_b0;
    w_b0;

    P0_c;
    V_c0;

    % eul_c0;
    e_c0;
    w_c0;
    ];

[t, y] = ode45(@(t, y) basic_parachute_dynamic_model(t, y, payload, parachute), 0:0.01:100, x0);

%% Plotting

plot_data(t, y, true, false, parachute)

function plot_data(t, y, do_animation, save_video, parachute)

if do_animation
figure(1)
clf
run_animation(t, y, 10, 10, save_video)
end

figure(2)
clf
plot(t, y(:, 4), 'DisplayName', 'X', 'LineWidth', 1.5); hold on;
plot(t, y(:, 5), 'DisplayName', 'Y', 'LineWidth', 1.5);
plot(t, y(:, 6), 'DisplayName', 'Z', 'LineWidth', 1.5);
plot(t, vecnorm(y(:,4:6), 2, 2), 'LineWidth', 3, 'DisplayName', 'Speed')

legend;
title("Body Frame Velocity vs. Time");
xlabel("Time (s)")
ylabel("Velocity (m/s)");

figure(3)
clf
yyaxis left
plot(t, y(:, 1), 'DisplayName', 'X', 'LineWidth', 1.5); hold on
plot(t, y(:, 2), 'DisplayName', 'Y', 'LineWidth', 1.5)
ylabel("X, Y Position (m)")

yyaxis right
plot(t, y(:,3), 'DisplayName', 'Z', 'LineWidth', 1.5)
ylabel("Z Position (m)")

title("ECEF Position vs. Time")
xlabel("Time (s)")
legend

figure(4)
clf
plot3(y(:, 1), y(:, 2), y(:, 3))

title("ECEF Trajectory")
xlabel("X")
ylabel("Y")
zlabel("Z")

figure(5)
clf
plot(t, y(:, 10), 'DisplayName', 'P', 'LineWidth', 1.5); hold on;
plot(t, y(:, 11), 'DisplayName', 'Q', 'LineWidth', 1.5)
plot(t, y(:, 12), 'DisplayName', 'R', 'LineWidth', 1.5)

xlabel("Time (s)")
ylabel("Angular Rate (rad/s)")
title("Angular Velocities vs. Time")
legend

figure(6)
clf
plot(t, wrapToPi(y(:, 7)), 'DisplayName', '\phi', 'LineWidth', 1.5); hold on;
plot(t, wrapToPi(y(:, 8)), 'DisplayName', '\theta', 'LineWidth', 1.5)
plot(t, wrapToPi(y(:, 9)), 'DisplayName', '\psi', 'LineWidth', 1.5)

xlabel("Time (s)")
ylabel("Angular Position (rad)")
title("Angular Position vs. Time")
legend

figure(7)
clf
plot(t, y(:, 3), 'DisplayName', 'Payload Height'); hold on;
plot(t, y(:, 16), 'DisplayName', 'Parachute Height'); hold on;
% xlim([0, 2])
legend

plot_Cd_vs_aoa(parachute);

plot_parachute_drag_components(t, y, parachute, @extract_VcB_rho);

end

function run_animation(t, y, step, substep, save_video)
numsteps = height(y);

quat = quaternion(y(1, 7:10));
patch = poseplot(quat); hold on
patch1 = poseplot(quaternion(y(1, 20:23)));

% patch.ScaleFactor = 50;
xlabel("X")
ylabel("Y")
zlabel("Z")

if save_video
outputVideo = VideoWriter('myVideo.mp4', 'MPEG-4'); % Specify filename and format
open(outputVideo);
end

for i = 2:step:numsteps - step
    for j=i:substep:i+step
        quat = quaternion(y(j, 7:10));
        pos = y(j, 1:3);
    
        set(patch, Orientation=quat, Position=pos); hold on
        set(patch1, Orientation=quaternion(y(j, 20:23)), Position=y(j, 14:16));

        legend("Payload", "Parachute")
        % plot3(pos(1), pos(2), pos(3), '.b', 'MarkerSize', 1); hold on
    end

    xlim(y(j,1) + [-1, 1]*20);
    ylim(y(j,2) + [-1, 1]*20);
    zlim(y(j,3) + [-1, 1]*20);

    set(gca,'ZDir','normal')  
    title(sprintf("t = %0.2f", t(i)))
    drawnow

    if save_video
    frame = getframe(gcf); % captures the current figure (gcf)
    
    writeVideo(outputVideo, frame);
    else
    % pause(0.05)
    end
end
if save_video
close(outputVideo)
end
end

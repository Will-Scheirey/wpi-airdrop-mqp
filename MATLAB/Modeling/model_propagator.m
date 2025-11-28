clear; clc; close all;

[t, y, model] = propagate_model('variable_parachute_mass', false, 'use_drag', false, 'damping', false, 'riser', false);

%% Plotting

% plot_data(t, y, true, false)

plot_energy(t, y, model.payload, model.parachute)

function plot_data(t, y, do_animation, save_video)

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

    set(gca,'ZDir','normal')  
    title(sprintf("t = %0.2f", t(i)))


    lim = 25;
    xlim(y(j,1) + [-1, 1]*lim);
    ylim(y(j,2) + [-1, 1]*lim);
    zlim(y(j,3) + [-1, 1]*lim);

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

function plot_energy(t, y, payload, parachute)
    payload_kinetic_energy = 0.5 *payload.m()*vecnorm(y(:,4:6), 2, 2).^2;
    parachute_kinetic_energy = 0.5 *parachute.m()*vecnorm(y(:,17:19), 2, 2).^2;
    
    omega = y(:, 11:13);
    payload_rotational_energy = 0.5 * sum((omega * payload.I()) .* omega, 2);
    
    omega_p = y(:, 24:26);
    parachute_rotational_energy = 0.5 * sum((omega_p * parachute.I()) .* omega_p, 2);    

    payload_potential_energy =   9.81 * payload.m() *y(:,3);
    parachute_potential_energy = 9.81 *parachute.m()*y(:,16);
    
    spring_length = zeros(height(y), 1);
    
    for idx = 1:height(y)
        e_p = y(idx, 7:10);
        rotm_p = ecef2body_rotm(e_p);
        P_a_p = y(idx,1:3)' + rotm_p' * payload.P_attach_B;
    
        e_c = y(idx, 20:23);
        rotm_c = ecef2body_rotm(e_c);
        P_a_c = y(idx,14:16)' + rotm_c' * parachute.P_attach_B;
    
        dist =  norm(P_a_p - P_a_c);
        extension = dist - parachute.l0;
    
        spring_length(idx) = extension;
    end
    
    spring_potential_energy = 0.5 * parachute.k_riser*spring_length.^2;
    
    payload_total = payload_kinetic_energy + payload_rotational_energy + payload_potential_energy;
    parachute_total = parachute_kinetic_energy + parachute_rotational_energy + parachute_potential_energy;
    
    figure(8)
    clf
    plot(t, payload_kinetic_energy, 'DisplayName', 'Payload Kinetic Energy', 'LineWidth', 1.5); hold on
    plot(t, parachute_kinetic_energy, 'DisplayName', 'Parachute Kinetic Energy', 'LineWidth', 1.5)
    hold on
    % plot(t, payload_rotational_energy, 'DisplayName', 'Payload Rotational Energy', 'LineWidth', 1.5)
    % plot(t, parachute_rotational_energy, 'DisplayName', 'Parachute Rotational Energy', 'LineWidth', 1.5)
    % plot(t, payload_rotational_energy + parachute_rotational_energy, 'DisplayName', 'Total Rotational Energy', 'LineWidth', 1.5)
    
    figure(9)
    clf
    total = payload_total + parachute_total + spring_potential_energy;
    % plot(t, payload_total, 'DisplayName', 'Payload Total Energy', 'LineWidth', 1.5); hold on
    % plot(t, parachute_total, 'DisplayName', 'Parachute Total Energy', 'LineWidth', 1.5)
    plot(t, (total(1) - total) / total(1), 'LineWidth', 1.5); hold on
    % plot(t, spring_potential_energy, 'DisplayName', 'Spring Potential Energy', 'LineWidth', 1.5)
    xlabel("Time (s)")
    ylabel("Lost Energy %")
    title("% Total Lost Energy / Initial Energy")
    % legend
    
    figure(10)
    clf
    plot(t, total)
    
    %{
    figure(9)
    clf
    plot(t, vecnorm(y(:, 7:10), 2, 2), 'DisplayName', 'Payload Quaternion Norm', 'LineWidth', 1.5); hold on
    plot(t, vecnorm(y(:, 20:23), 2, 2), 'DisplayName', 'Parachute Quaternion Norm', 'LineWidth', 1.5)
    
    legend
    %}
end

function run_animation(t, y, step, substep, save_video)
numsteps = height(y);

quat = quaternion(y(1, 7:10));
patch = poseplot(quat); hold on
patch1 = poseplot(quaternion(y(1, 20:23)));

scaleFactor = 2;

patch.ScaleFactor = scaleFactor;
patch1.ScaleFactor = scaleFactor;

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
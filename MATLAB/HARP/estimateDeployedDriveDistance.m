function ddd_m = estimateDeployedDriveDistance(deployedDriveTime_s, deployed_ARoF_fps)
% Estimate deployed drive distance using empirical relationship: forward speed ~ some fraction of deployed RoF
% This is a placeholder: real deployed drive distance requires para forward speed (which depends on canopy and K-factor).
% Here we estimate forward velocity = 0.6 * deployed ARoF (ft/s) as an example (not authoritative).
forwardVel_fps = 0.6 * deployed_ARoF_fps;
ddd_ft = forwardVel_fps * deployedDriveTime_s;
ddd_m = ddd_ft * 0.3048;
end

function driveTime_s = deployedDriveTime(totalDeployedTime_s, DT_s)
% deployedDriveTime - deployed drive time approximated as total deployed time minus deceleration time
% AFMAN Figure 5.14/5.16 references DT (deceleration time) and DRIVE time
driveTime_s = max(0, totalDeployedTime_s - DT_s);
end

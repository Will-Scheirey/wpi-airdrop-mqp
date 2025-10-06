function total_s = computeTotalDeployedTime(absActuationAlt_ft, VD_ft, deployed_ARoF_fps)
% total deployed time = (absActuationAlt - 0) / Deployed ARoF
% but AFMAN Figure 5.13 may specify the total deployed time from actuation to impact.
fallDist_ft = max(0, absActuationAlt_ft); % distance from actuation to PI ground (ft)
if deployed_ARoF_fps <= 0
    total_s = NaN;
else
    total_s = fallDist_ft / deployed_ARoF_fps;
end
end

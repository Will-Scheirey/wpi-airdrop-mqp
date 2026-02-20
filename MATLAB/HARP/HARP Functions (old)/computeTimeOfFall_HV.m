function ToF_s = computeTimeOfFall_HV(stabAlt_ft, absActuationAlt_ft, ARoF_fps, TFC_s)
% computeTimeOfFall_HV - compute HV time of fall
% AFMAN Figure 5.8/4.6 use ToF formula E (distance / ARoF) plus TFC where applicable.
% We compute time from stabilization altitude down to actuation (both AGL). If actuation
% altitude is above stabilization altitude, ToF is truncated at zero.
fallDistance_ft = max(0, (stabAlt_ft - absActuationAlt_ft));
if ARoF_fps <= 0
    ToF_s = NaN;
else
    ToF_s = fallDistance_ft / ARoF_fps;
end
% AFMAN uses TFC as a separate additive constant to total time of fall (Item 18).
% Here we return only the HV ToF (Item 16). The full total time will include TFC elsewhere.
end

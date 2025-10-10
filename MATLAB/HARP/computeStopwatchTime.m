function stopwatchTime_s = computeStopwatchTime(stopwatchDistance_m, FTD_m, IAS_KIAS, pressureAlt_ft)
% stopwatch time = stopwatchDistance / groundspeed
% If stopwatchDistance is NaN, return NaN.
if isnan(stopwatchDistance_m)
    stopwatchTime_s = NaN;
    return;
end
% estimate groundspeed from IAS and pressure altitude
TAS_kt = IAS2TAS(IAS_KIAS, pressureAlt_ft);
gs_ms = TAS_kt * 0.514444;
stopwatchTime_s = stopwatchDistance_m / gs_ms;
end

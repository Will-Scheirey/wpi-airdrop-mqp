function t_s = distanceToTime(dist_m, IAS_KIAS, pressureAlt_ft)
% Convert a distance to time by dividing by groundspeed (estimated from IAS->TAS)
TAS_kt = IAS2TAS(IAS_KIAS, pressureAlt_ft);
gs_ms = TAS_kt * 0.514444;
t_s = dist_m / gs_ms;
end

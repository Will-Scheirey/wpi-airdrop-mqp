function FTD_m = computeFTD(FTT_s, IAS_KIAS, pressureAlt_ft)
% computeFTD - forward travel distance = groundspeed * FTT
% We approximate groundspeed from IAS -> TAS using standard atmosphere approximation
TAS_kt = IAS2TAS(IAS_KIAS, pressureAlt_ft);
% convert kt to m/s
ms = TAS_kt * 0.514444;
FTD_m = ms * FTT_s;
end

function GS_kt = estimateGroundspeed(IAS_KIAS, pressureAlt_ft, tempC)
% estimateGroundspeed from IAS (approx TAS) and neglect wind for a simple estimate
if nargin < 3
    tempC = 15;
end
TAS_kt = IAS2TAS(IAS_KIAS, pressureAlt_ft);
GS_kt = TAS_kt; % crude: assumes no wind or small head/tail
end

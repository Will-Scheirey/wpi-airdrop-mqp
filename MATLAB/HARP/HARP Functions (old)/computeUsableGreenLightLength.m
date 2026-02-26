function length_m = computeUsableGreenLightLength(DDD_m, DWE_m)
% computeUsableGreenLightLength - approximate green light length = deployed drive distance +/- wind effect
% This is mission dependent; we return an indicative length as DDD + |DWE|
length_m = abs(DDD_m) + abs(DWE_m);
end

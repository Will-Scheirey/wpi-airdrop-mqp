function midPres_ft = hvMidPressureAlt(dropPressureAlt_ft, actuationPressureAlt_ft)
% HV mid pressure altitude - midpoint
midPres_ft = (dropPressureAlt_ft + actuationPressureAlt_ft) / 2;
end

function dmpa_ft = deployedMidPressureAlt(actuationPressureAlt_ft, PI_pressureAlt_ft)
% midpoint between actuation pressure altitude and PI pressure altitude
dmpa_ft = (actuationPressureAlt_ft + PI_pressureAlt_ft) / 2;
end

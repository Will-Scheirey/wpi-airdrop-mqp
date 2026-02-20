function actTrueAlt_ft = computeActuationTrueAlt(actAGL_ft, PI_elev_ft)
% actTrueAlt = actuation indicated altitude (AGL) + PI elevation (MSL)
actTrueAlt_ft = actAGL_ft + PI_elev_ft;
end

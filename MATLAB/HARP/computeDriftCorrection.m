function driftCorr_deg = computeDriftCorrection(ballisticWindDir_deg, DZ_course_deg)
% compute drift correction heading to parallel DZ course
% If ballistic wind is from ~0 deg, drift pushes downwind along its direction; drift correction is difference
driftAngle = ballisticWindDir_deg - DZ_course_deg;
% Usually the required run-in heading correction is small; store difference as a signed angle
driftCorr_deg = driftAngle; 
end

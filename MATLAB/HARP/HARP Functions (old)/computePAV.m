function PAV_ft = computePAV(altimeterSetting_inHg)
% computePAV - Pressure Altitude Variation (ft) using AFMAN formula A
% PAV = (29.92 - altimeterSetting) * 1000  (positive if setting < 29.92)
PAV_ft = (29.92 - altimeterSetting_inHg) * 1000;
end

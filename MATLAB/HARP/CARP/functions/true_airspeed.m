function v = true_airspeed(equivalent_airspeed,pressure_altitude,true_altitude_temperature)
    v = (equivalent_airspeed * true_altitude_temperature)/pressure_altitude; 
end
#!/bin/zsh

cd /Applications/FlightGear.app/Contents/Resources/data

export FG_ROOT=/Applications/FlightGear.app

/Applications/FlightGear.app/Contents/MacOS/FlightGear --fdm=null --native-fdm=socket,in,30,localhost,5502,udp  --enable-terrasync --prop:/sim/rendering/shaders/quality-level=0 --aircraft=Parachutist --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --airport=KSFO --runway=10L --altitude=7224 --heading=113 --offset-distance=4.72 --offset-azimuth=0   &

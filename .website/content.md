thallos is an experimental system built by me, dhruv and aayan for mapping fine-scale environmental variation using a compact sensor payload mounted on a drone. instead of assuming a field or environment is uniform, thallos measures small differences in temperature, humidity, canopy heat, and air composition. these shifts often hint at plant stress, dryness, or uneven growth long before anything is visible.

the core idea is simple: a consumer drone, a lightweight sensor stack, and a clean processing pipeline can produce meaningful stress maps without specialized agricultural hardware.

## test flights

test flight — pre-autonomous mapping  
test flight — autonomous mapping  
![autonomous mapping](https://github.com/Shaileshsaravanan/thallos/blob/main/content/auto_thumb.png?raw=true)

![top view](https://github.com/Shaileshsaravanan/thallos/blob/main/content/topview.jpg?raw=true)

## how the system works

the drone follows a predefined mission using [ArduPilot](https://ardupilot.org/) / [Mission Planner](https://ardupilot.org/planner/). at each sampling waypoint, an onboard [Raspberry Pi](https://www.raspberrypi.com/) records:

• air temperature  
• relative humidity  
• infrared canopy temperature  
• estimated co₂ / tvoc  
• optional rgb frame  
• gps position  

the waypoint files (.mission / .kml) are later used for alignment and visualization, so each stress value maps exactly to where it was recorded. the drone effectively acts as a mobile microclimate probe, collecting spatially precise snapshots across an area.

![side view](https://github.com/Shaileshsaravanan/thallos/blob/main/content/sideview.jpg?raw=true)

## current sensor prototype

**DHT11 / SHT31 — air temperature + humidity**  
used to compute vpd (vapor pressure deficit), a strong indicator of atmospheric dryness and crop stress.

high vpd → higher water loss → more stress  
low vpd → cooler, humid → less stress  

**MLX90614 — canopy infrared temperature**  
warmer canopy often points to water stress and early stomatal closure.

**SGP30 — eco₂ / tvoc estimate**  
helps identify ventilation pockets, vegetation density effects, and trapped air sections.

**rgb camera**  
provides visual context like shading and general greenness. not used for deep vision ml in this prototype.

## data pipeline

• drone executes mission  
• sensors log data at hover points → csv rows  
• features extracted:  
  – vpd  
  – canopy vs air temperature delta  
  – shading estimate  
  – normalized eco₂ deviation  
  – optional greenness score  
• microclimate stress index (msi, 0–1) computed  
• visualization uses [Leaflet](https://leafletjs.com/) heatmaps + KML overlays ([KML Reference](https://developers.google.com/kml/documentation))  

red = high stress  
yellow = moderate  
green = low  

optional rule-based text summaries explain why stress is detected in each region.

## processing and ml approach

the drone handles data capture. processing happens on a backend server, which keeps flight time higher and inference faster. earlier attempts at full onboard processing on raspberry pi 4 worked, but power cost was high. offloading gives better performance.

### server-side prediction

**air quality estimation**  
distinguishes stagnation / pollution pockets vs heat-driven stress.

**temperature + humidity forecasting**  
short-term microclimate prediction to highlight early stress trends based on vpd and canopy-air deltas.

**spatial estimation**
interpolation (idw / kriging) fills gaps between points to create continuous heatmaps rather than isolated dots.

data flow looks like this:

drone collects → sends to server → server runs models → ui updates in real time.

models can improve without changing the drone hardware.

## key capabilities

• drone-based microclimate sensing  
• sensor fusion across heat, humidity, and air composition  
• interpretable microclimate stress index  
• high-resolution heatmap visualization  
• kml flight path overlays  
• clustering + interpolation  
• low-cost, modular hardware

## purpose

thallos is a proof-of-concept that low-cost hardware can capture meaningful environmental variation at a high resolution. prototype results show useful stress differences across fields, parks, and landscaped environments.

this kind of data can support:

• localized irrigation  
• shade planning  
• vegetation + soil health monitoring  
• research and environmental analysis
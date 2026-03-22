# Intersection Simulator

A browser-based signalized intersection simulator built for CEE 370 Transportation Fundamentals.

This project visualizes traffic operations at a four-leg intersection with:

- one inbound lane on each approach
- a two-phase traffic signal
- stochastic vehicle arrivals by approach
- left, through, and right turning movements
- live queue, delay, arrival, and departure statistics
- a classroom-friendly control panel and animated canvas display

## Features

- Adjustable east-west green, yellow clearance, and north-south green times
- Separate arrival-rate inputs for each approach
- Adjustable free-flow speed
- Adjustable large-vehicle percentage
- Adjustable simulation speed
- Real-time signal status, clock, and performance metrics
- Animated vehicle movement with basic conflict handling for permissive left turns

## Project Files

- `index.html` - page structure, controls, canvas, and statistics layout
- `styles.css` - visual styling for the interface
- `app.js` - simulation logic, signal timing, arrivals, vehicle movement, and rendering

## How To Run

No build step is required.

1. Open `index.html` in a web browser.
2. Adjust the control values as needed.
3. Click `Start` to run the simulation.
4. Use `Pause` to stop the animation and `Reset` to restart with the current settings.

## Controls

- `Phase 1: East-West Green (s)` - green time for east-west traffic
- `Yellow / Clearance (s)` - clearance interval between phases
- `Phase 2: North-South Green (s)` - green time for north-south traffic
- `Arrival rates` - separate demand levels for each approach in vehicles per hour
- `Free-Flow Speed (mph)` - target speed used by simulated vehicles
- `Large Vehicle Ratio (%)` - share of larger vehicles in the traffic stream
- `Simulation Speed` - playback speed multiplier for the simulation

## Notes

- The simulator is intended for teaching and demonstration rather than detailed traffic operations analysis.
- Vehicle arrivals are generated stochastically.
- Left turns use a permissive movement model and yield to opposing through traffic when needed.
- Setting an approach arrival rate to `0` disables arrivals from that direction.

## Author

Dr. Kun Xie  
Old Dominion University  
`kxie@odu.edu`

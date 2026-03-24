# Intersection Simulator

A browser-based signalized intersection simulator developed for CEE 370 Transportation Fundamentals at Old Dominion University.

The project combines:

- a live browser simulation with an animated intersection view
- a matching offline analysis script for repeated scenario testing
- paper assets that document the model, control logic, and classroom use cases

## What The Simulator Models

The simulator represents a four-leg signalized intersection with:

- one inbound lane on each approach
- stochastic vehicle arrivals by approach
- left, through, and right turning movements
- pedestrian crossings on all approaches
- a fixed two-phase controller with green, yellow, and all-red intervals
- live measures of arrivals, departures, delay, queue, and signal status

The model is intended for instruction and demonstration rather than field calibration.

## Current Features

- Adjustable east-west green, north-south green, yellow, and all-red times
- Separate approach demand inputs in vehicles per hour
- Adjustable total pedestrian demand
- Adjustable free-flow speed
- Adjustable left-turn, right-turn, and large-vehicle percentages
- Real-time queue, average queue, maximum queue, delay, arrivals, and departures
- Permissive left-turn logic with targeted yellow clearance for lead left-turners
- Conflict handling between permissive left turns and opposing through traffic
- Conflict handling between permissive left turns and opposing right turns
- Pedestrian entry screening, crosswalk occupancy, and simple pedestrian dodging
- Vehicle yielding to pedestrians in and near the crosswalk/intersection area
- Offline scenario analysis through `tools/sim_analysis.js`

## Main Files

- `index.html` - simulator layout, controls, canvas, and statistics panel
- `styles.css` - visual styling for the web interface
- `app.js` - browser simulation logic, signal control, vehicle/pedestrian behavior, and rendering
- `tools/sim_analysis.js` - headless analysis runner for repeated scenario evaluation
- `WHITE_PAPER.md` - white paper overview of the simulator
- `conference_paper.tex` - conference-style LaTeX paper
- `conference_paper.pdf` - compiled conference paper

## How To Run The Browser Simulator

No build step is required.

1. Open `index.html` in a web browser.
2. Adjust the input values in the control panel.
3. Click `Start` to begin the simulation.
4. Use `Pause` to stop the animation and `Reset` to restart with the current settings.

## Controls

- `Phase 1: East-West Green (s)` - green time for the east-west phase
- `Phase 2: North-South Green (s)` - green time for the north-south phase
- `Yellow (s)` - yellow interval for the active phase
- `All-Red (s)` - clearance interval between phase groups
- `Eastbound / Westbound / Southbound / Northbound Arrival (veh/hr)` - approach-specific vehicle demand
- `Total Pedestrian Volume (ped/hr)` - total pedestrian demand distributed across approaches
- `Free-Flow Speed (mph)` - target vehicle speed
- `Left Turn Ratio (%)` - proportion of vehicles assigned to left turns
- `Right Turn Ratio (%)` - proportion of vehicles assigned to right turns
- `Large Vehicle Ratio (%)` - proportion of larger vehicles in the traffic stream
- `Simulation Speed` - playback speed multiplier

## Run Offline Analysis

The analysis script can be executed directly with Node.js:

```bash
node tools/sim_analysis.js
```

This prints a JSON summary of the predefined scenarios, including mean:

- average delay
- average queue
- maximum queue
- completed trips
- throughput

To write the results to a file:

```bash
node tools/sim_analysis.js output/results.json
```

## Paper And Documentation Assets

The repository also contains written documentation for teaching and dissemination:

- `WHITE_PAPER.md` introduces the simulator and summarizes the model logic
- `conference_paper.tex` and `conference_paper.pdf` present the simulator in conference-paper format
- the paper includes scenario comparisons, performance metrics, figures, references, and pseudocode for key control and crash-avoidance logic

To rebuild the conference paper locally:

```bash
pdflatex -interaction=nonstopmode -halt-on-error conference_paper.tex
```

Run `pdflatex` twice if cross-references need to be refreshed.

## Notes

- The browser simulator and `tools/sim_analysis.js` are intended to stay behaviorally aligned.
- Scenario results from the analysis script are illustrative outputs from the teaching simulator, not calibrated operational estimates.
- Setting an arrival rate or pedestrian volume to `0` disables that demand source.

## Author

Dr. Kun Xie  
Old Dominion University  
`kxie@odu.edu`

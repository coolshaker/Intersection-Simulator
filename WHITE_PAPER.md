# Intersection Simulator White Paper

## A Browser-Based Teaching Simulator for Signal Control, Vehicle Interaction, and Pedestrian Safety Behavior

### Abstract

This white paper introduces the Intersection Simulator developed for CEE 370 Transportation Fundamentals at Old Dominion University. The simulator is a browser-based instructional tool that visualizes traffic operations at a four-leg signalized intersection with one inbound lane per approach. Its primary purpose is to help students connect traffic engineering concepts, such as signal timing, queue formation, delay, permissive left turns, and pedestrian conflicts, to observable operational behavior.

The simulator is not intended to be a calibrated field-analysis package. Instead, it is structured as a compact, transparent, and modifiable teaching model. Its value lies in how clearly it represents the interaction among signal control, stochastic arrivals, car-following behavior, turning maneuvers, and pedestrian movements. Particular emphasis is placed on conflict management and crash-avoidance behavior. Vehicles respond to signals, stop lines, leading vehicles, pedestrians, and opposing traffic during permissive left turns. Pedestrians respond to signal opportunity, vehicle presence near the crosswalk, and local obstructions during crossing.

This document focuses especially on the simulator's control logic and its modeled crash-avoidance mechanisms for both vehicles and pedestrians.

## 1. Introduction

Traffic engineering instruction often begins with deterministic equations for saturation flow, Webster-style signal timing, queueing, and delay. While these methods are essential, they do not always help students visualize how traffic systems behave moment by moment. The Intersection Simulator addresses that gap by providing a real-time animated environment in which students can vary signal timing, demand, vehicle mix, and movement proportions, then observe the resulting operational patterns.

The simulator represents a four-leg signalized intersection with:

- one inbound lane on each approach
- east-west and north-south phase groups
- user-adjustable green, yellow, and all-red intervals
- stochastic vehicle arrivals by approach
- left, through, and right turning movements
- a pedestrian crossing model at each approach
- live performance metrics for queue, arrivals, departures, and delay

Because the simulator is designed for classroom use, the model balances realism and clarity. It includes important operational interactions without becoming so complex that the core logic is obscured.

## 2. System Overview

The simulator is implemented as a browser-based JavaScript application with a canvas-based animation layer. The same core behavioral concepts are mirrored in the analysis script used for scenario testing. At a high level, the simulation loop performs the following tasks repeatedly:

1. Read or retain user-defined configuration values.
2. Advance the signal state based on elapsed time.
3. Generate vehicle and pedestrian arrivals stochastically.
4. Update vehicle behavior on approaches and within the intersection.
5. Update pedestrian movement and pedestrian-vehicle interactions.
6. Recompute queue and delay statistics.
7. Render the updated system state.

This structure makes the simulator useful both as an interactive visualization tool and as a compact analytical engine for repeated scenario evaluation.

## 3. Signal Control Logic

### 3.1 Phase Structure

The simulator uses a two-phase control structure:

- `ew-green`
- `ew-yellow`
- `ew-all-red`
- `ns-green`
- `ns-yellow`
- `ns-all-red`

East-west movements and north-south movements are assigned to separate phase groups. The current signal duration is determined by the active state:

- green states use the configured green time for the corresponding phase group
- yellow states use a common user-defined yellow interval
- all-red states use a common user-defined all-red interval

This design gives students a direct way to examine the effects of cycle structure, clearance timing, and green allocation.

### 3.2 Movement Permission

A vehicle is permitted to proceed if its approach belongs to the currently active green phase. The simulator also includes a targeted yellow-clearance rule for permissive left turns. When a phase changes from green to yellow, the lead left-turn vehicle on each active approach may be tagged for yellow completion. This allows a queued left-turn vehicle that has been waiting for an acceptable gap to finish the maneuver during the internal yellow interval, rather than unnecessarily blocking the entire queue.

This behavior is intentionally narrow. The yellow interval does not open the intersection to the entire queue. It only allows the specific lead left-turn vehicle captured at the green-to-yellow transition to use that clearance opportunity.

### 3.3 Operational Interpretation

Pedagogically, this controller shows that even a simple two-phase signal can produce a range of outcomes depending on:

- demand imbalance
- turning proportions
- clearance timing
- the interaction between permissive movements and conflicting streams

The simulator therefore supports discussions not only of fixed timing, but also of phase utilization and lost time.

## 4. Vehicle Motion Logic

### 4.1 Arrival Generation

Vehicles arrive on each approach according to an exponential headway process. This creates random arrivals around a user-defined hourly rate and avoids an unrealistically uniform stream. Students can therefore observe how queues fluctuate even under constant average demand.

### 4.2 Movement Assignment

Each generated vehicle is assigned a movement:

- left
- through
- right

The distribution is controlled by user-defined turning proportions. Vehicles are also assigned a desired speed and a size class, allowing the simulation to include a mix of passenger vehicles and larger vehicles.

### 4.3 Car-Following

Approach vehicles use a simplified Intelligent Driver Model style acceleration rule. For each vehicle, the simulator computes a lead object when relevant:

- the vehicle directly ahead in the queue
- a same-approach vehicle that has just entered the intersection
- a stop condition imposed by the signal
- a stop condition imposed by a pedestrian conflict
- a stop condition imposed by a permissive turning conflict

Acceleration is then determined from the vehicle's desired speed, the current speed, the speed difference to the lead object, and the available gap. This produces smoother deceleration and acceleration than a purely rule-based stop-go scheme.

### 4.4 Queueing

Queue count is estimated in real time from vehicles near the stop line that are creeping or stopped, together with followers trapped behind them. This gives students immediate feedback on queue growth, dissipation, and spillback risk.

## 5. Crash-Avoidance Logic for Vehicles

The simulator's vehicle safety behavior is built around conflict anticipation rather than post hoc collision repair. Several mechanisms work together to keep vehicles separated.

### 5.1 Signal and Stop-Line Compliance

If a vehicle is not permitted by signal state, a stationary control point is introduced at the stop line and used as the lead object in the car-following calculation. This causes vehicles to decelerate into the stop bar rather than teleporting or stopping instantaneously.

### 5.2 Same-Approach Rear-End Avoidance

Vehicles maintain separation from leaders in the approach queue using gap calculations based on vehicle length, travel direction, and front-to-rear spacing. The same concept is extended to the intersection entry zone: a vehicle on the approach can still recognize a same-approach vehicle that has just entered the intersection and avoid following too closely.

### 5.3 Permissive Left-Turn Yielding to Opposing Through Traffic

A left-turn vehicle is assigned a conflict profile based on where its turning path intersects the opposing through path. Before entering the intersection, the left-turn vehicle checks whether an opposing through vehicle is close enough to that conflict point to create a hazard. If so, the left-turn vehicle treats the stop line as a temporary yielding boundary.

This check continues inside the intersection. If the left-turn vehicle has entered but has not yet cleared the conflict region, it can decelerate and pause at a predefined yield point. This reflects the common field condition in which a driver enters permissively, waits in the intersection, and completes the turn once opposing traffic has passed.

### 5.4 Permissive Left-Turn Yielding to Opposing Right Turns

The simulator also addresses a second conflict that is often overlooked in simple educational models: the shared receiving lane conflict between a permissive left turn and an opposing right turn. The code estimates where the left-turn path and the opposing right-turn path converge into the same outbound lane and uses that location to generate another yield profile.

As a result, a left-turn vehicle can be held:

- before entering the intersection if an opposing right-turn vehicle is about to occupy the shared exit lane
- within the intersection if the left-turn vehicle would otherwise merge into that lane unsafely

This is an important feature because it prevents visually obvious path overlap and reinforces the principle that permissive left turns must account for more than one conflicting movement.

### 5.5 Route-Based Separation Inside the Intersection

Once vehicles are in the intersection, they are grouped by route and ordered by progress along that route. Followers are then prevented from exceeding the progress of the leader minus a minimum gap. This reduces the chance of overlap among vehicles taking the same movement and maintains visual and operational continuity through the intersection.

## 6. Pedestrian Control and Crash-Avoidance Behavior

Pedestrian behavior is also modeled explicitly, both at entry to the crosswalk and during the crossing itself.

### 6.1 Pedestrian Demand and Release

Pedestrian requests are generated stochastically and stored as pending crossing demand. A pedestrian begins crossing only when:

- the corresponding vehicle approach is not green, and
- the lead vehicle near the stop line is either absent, far enough away, or effectively stopped

This creates a simplified but intuitive pedestrian opportunity rule. Pedestrians are not inserted into the crosswalk if a vehicle is still too close to the crossing entry point.

### 6.2 Multiple Pedestrians and Lane Offsets

Pedestrians are assigned lateral offsets within the crosswalk so that more than one pedestrian can occupy the crossing without collapsing into a single centerline. Spawn spacing checks prevent multiple pedestrians from appearing on top of one another at the entry zone.

### 6.3 In-Crosswalk Obstacle Avoidance

Each pedestrian forecasts the next intended position along the crosswalk. If that position conflicts with a vehicle envelope, the pedestrian searches for a lateral dodge offset. Several candidate offsets are evaluated, including preferred and opposite-side bypass options. If a safe forward dodge exists, the pedestrian proceeds with lateral deviation. If not, the pedestrian may hold position and attempt a lateral reposition.

This produces a simple but effective local avoidance behavior that is especially useful when a vehicle nose partially encroaches near the crosswalk or when a stopped large vehicle creates a visual blockage.

### 6.4 Vehicle Yielding to Pedestrians

Vehicle crash-avoidance also accounts for pedestrians. For vehicles already inside the intersection, the simulator identifies the nearest pedestrian conflict by projecting pedestrian position onto the vehicle's path. If the pedestrian lies sufficiently near the vehicle trajectory and ahead of the vehicle's current progress point, the vehicle is assigned a stop progress before the conflict zone. The vehicle then decelerates and holds until the pedestrian clears.

This mechanism turns pedestrian safety into an active part of the vehicle decision process rather than treating pedestrians as purely visual actors.

## 7. Educational Value

The simulator is especially useful for demonstrating:

- the relationship between green allocation and queue growth
- how stochastic arrivals create variable performance even under fixed timing
- why permissive left turns can become a dominant source of delay
- how multiple conflict types affect left-turn completion
- why pedestrian activity influences vehicle progression and saturation
- the difference between capacity, delay, and observed operational smoothness

Because the code is compact and readable, it also supports instructional use in transportation computing, introductory simulation, and algorithmic traffic control courses.

## 8. Limitations

The simulator deliberately simplifies several aspects of real-world operation:

- one inbound lane is used for all movements on each approach
- the controller is fixed-time rather than actuated or adaptive
- driver behavior is stylized and does not represent full heterogeneity
- pedestrian behavior is rule-based rather than agent-calibrated
- safety logic is conflict-driven and instructional, not surrogate-safety calibrated

These limitations are appropriate for a classroom-focused model but should be recognized if the simulator is used for analysis beyond teaching and demonstration.

## 9. Future Development Opportunities

Several extensions would strengthen the simulator's educational and analytical range:

- protected and protected-permissive left-turn phasing
- multiple lanes and lane-use assignment
- approach-specific turning proportions
- pedestrian signal phasing and walk/flashing-don't-walk logic
- red-light compliance variability
- richer safety surrogate measures such as post-encroachment time or conflict counts
- scenario export and automated comparative reporting

## 10. Conclusion

The Intersection Simulator demonstrates that a compact browser-based model can still capture the core dynamics that matter for teaching signalized intersection operations. Its signal controller, stochastic arrivals, car-following logic, permissive left-turn treatment, and pedestrian interaction rules together create a system that is simple enough for classroom use yet rich enough to illustrate important transportation engineering concepts.

Most importantly, the simulator does not treat vehicles and pedestrians as passive animations. It incorporates explicit crash-avoidance behavior through signal compliance, queue following, conflict detection, yielding rules, pedestrian screening, and in-crosswalk obstacle avoidance. That makes it a useful educational bridge between static equations and the dynamic, conflict-rich environment of real intersection operations.

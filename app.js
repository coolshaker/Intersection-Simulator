const canvas = document.getElementById("sim-canvas");
const ctx = canvas.getContext("2d");

const controls = {
  green: document.getElementById("green-duration"),
  yellow: document.getElementById("yellow-duration"),
  red: document.getElementById("red-duration"),
  arrivalRateWest: document.getElementById("arrival-rate-west"),
  arrivalRateEast: document.getElementById("arrival-rate-east"),
  arrivalRateNorth: document.getElementById("arrival-rate-north"),
  arrivalRateSouth: document.getElementById("arrival-rate-south"),
  desiredSpeed: document.getElementById("desired-speed"),
  leftTurnRatio: document.getElementById("left-turn-ratio"),
  largeVehicleRatio: document.getElementById("large-vehicle-ratio"),
  simSpeed: document.getElementById("sim-speed"),
  simSpeedLabel: document.getElementById("sim-speed-label"),
  start: document.getElementById("start-button"),
  pause: document.getElementById("pause-button"),
  reset: document.getElementById("reset-button"),
};

const statEls = {
  arrivals: document.getElementById("arrivals-stat"),
  departures: document.getElementById("departures-stat"),
  queue: document.getElementById("queue-stat"),
  maxQueue: document.getElementById("max-queue-stat"),
  delay: document.getElementById("delay-stat"),
  signal: document.getElementById("signal-stat"),
  status: document.getElementById("sim-status"),
  clock: document.getElementById("clock-display"),
};

const pxPerMeter = 5.2;
const roadCenterX = 520;
const roadCenterY = 280;
const roadHalfWidth = 60;
const approachLength = 420;
const exitLength = 220;
const minVehicleGapPx = 12.0 * pxPerMeter + 8;  // Based on max vehicle length (12m) + buffer
const opposingThroughYieldDistancePx = 190;
const leftTurnYieldBufferPx = 18;
const defaultMovementRatios = {
  left: 0.23,
  through: 0.53,
  right: 0.24,
};

function point(x, y) {
  return { x, y };
}

function distanceBetween(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

const approaches = {
  west: {
    key: "west",
    label: "Eastbound",
    axis: "x",
    phaseGroup: "ew",
    spawnCoord: -approachLength,
    stopCoord: roadCenterX - roadHalfWidth - 10,
    travelSign: 1,
    queueOrder: (a, b) => b.coord - a.coord,
    pathDirection: 0,
    laneCenter: roadCenterY + 30,
    signalPos: point(roadCenterX - 92, roadCenterY + 90),
  },
  east: {
    key: "east",
    label: "Westbound",
    axis: "x",
    phaseGroup: "ew",
    spawnCoord: canvas.width + approachLength,
    stopCoord: roadCenterX + roadHalfWidth + 10,
    travelSign: -1,
    queueOrder: (a, b) => a.coord - b.coord,
    pathDirection: Math.PI,
    laneCenter: roadCenterY - 30,
    signalPos: point(roadCenterX + 92, roadCenterY - 90),
  },
  north: {
    key: "north",
    label: "Southbound",
    axis: "y",
    phaseGroup: "ns",
    spawnCoord: -approachLength,
    stopCoord: roadCenterY - roadHalfWidth - 10,
    travelSign: 1,
    queueOrder: (a, b) => b.coord - a.coord,
    pathDirection: Math.PI / 2,
    laneCenter: roadCenterX - 30,
    signalPos: point(roadCenterX - 92, roadCenterY - 90),
  },
  south: {
    key: "south",
    label: "Northbound",
    axis: "y",
    phaseGroup: "ns",
    spawnCoord: canvas.height + approachLength,
    stopCoord: roadCenterY + roadHalfWidth + 10,
    travelSign: -1,
    queueOrder: (a, b) => a.coord - b.coord,
    pathDirection: -Math.PI / 2,
    laneCenter: roadCenterX + 30,
    signalPos: point(roadCenterX + 92, roadCenterY + 90),
  },
};

const approachList = Object.values(approaches);

const movementTargets = {
  west: {
    left: point(roadCenterX + 30, -exitLength),
    through: point(canvas.width + exitLength, approaches.west.laneCenter),
    right: point(roadCenterX - 30, canvas.height + exitLength),
  },
  east: {
    left: point(roadCenterX - 30, canvas.height + exitLength),
    through: point(-exitLength, approaches.east.laneCenter),
    right: point(roadCenterX + 30, -exitLength),
  },
  north: {
    left: point(canvas.width + exitLength, roadCenterY + 30),
    through: point(approaches.north.laneCenter, canvas.height + exitLength),
    right: point(-exitLength, roadCenterY - 30),
  },
  south: {
    left: point(-exitLength, roadCenterY - 30),
    through: point(approaches.south.laneCenter, -exitLength),
    right: point(canvas.width + exitLength, roadCenterY + 30),
  },
};

const turnControlPoints = {
  west: {
    left: [
      point(roadCenterX - 10, approaches.west.laneCenter),
      point(roadCenterX + 30, roadCenterY + 18),
    ],
    right: [
      point(roadCenterX - 4, approaches.west.laneCenter),
      point(roadCenterX - 30, roadCenterY + 42),
    ],
  },
  east: {
    left: [
      point(roadCenterX + 10, approaches.east.laneCenter),
      point(roadCenterX - 30, roadCenterY - 18),
    ],
    right: [
      point(roadCenterX + 4, approaches.east.laneCenter),
      point(roadCenterX + 30, roadCenterY - 42),
    ],
  },
  north: {
    left: [
      point(approaches.north.laneCenter, roadCenterY - 10),
      point(roadCenterX - 18, roadCenterY + 30),
    ],
    right: [
      point(approaches.north.laneCenter, roadCenterY - 4),
      point(roadCenterX - 42, roadCenterY - 30),
    ],
  },
  south: {
    left: [
      point(approaches.south.laneCenter, roadCenterY + 10),
      point(roadCenterX + 18, roadCenterY - 30),
    ],
    right: [
      point(approaches.south.laneCenter, roadCenterY + 4),
      point(roadCenterX + 42, roadCenterY + 30),
    ],
  },
};

const sim = {
  running: false,
  lastTimestamp: 0,
  time: 0,
  vehicles: [],
  nextVehicleId: 1,
  nextArrivalTimes: {},
  stats: {
    arrivals: 0,
    departures: 0,
    currentQueue: 0,
    maxQueue: 0,
    totalDelay: 0,
  },
  signal: {
    state: "ew-green",
    elapsed: 0,
  },
  config: readConfig(),
};

function readConfig() {
  return {
    eastWestGreen: clampNumber(controls.green.value, 5, 120, 18),
    yellow: clampNumber(controls.yellow.value, 2, 20, 4),
    northSouthGreen: clampNumber(controls.red.value, 5, 120, 20),
    arrivalRatePerHour: {
      west: clampNumber(controls.arrivalRateWest.value, 0, 1800, 600),
      east: clampNumber(controls.arrivalRateEast.value, 0, 1800, 600),
      north: clampNumber(controls.arrivalRateNorth.value, 0, 1800, 600),
      south: clampNumber(controls.arrivalRateSouth.value, 0, 1800, 600),
    },
    desiredSpeedMps: clampNumber(controls.desiredSpeed.value, 10, 45, 28) * 0.44704,
    leftTurnRatio: clampNumber(controls.leftTurnRatio.value, 0, 100, 23) / 100,
    largeVehicleRatio: clampNumber(controls.largeVehicleRatio.value, 0, 100, 10) / 100,
    simSpeed: clampNumber(controls.simSpeed.value, 0.5, 10, 1),
  };
}

function clampNumber(value, min, max, fallback) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) {
    return fallback;
  }
  return Math.min(max, Math.max(min, parsed));
}

function formatClock(seconds) {
  const mins = Math.floor(seconds / 60);
  const secs = Math.floor(seconds % 60);
  return `${String(mins).padStart(2, "0")}:${String(secs).padStart(2, "0")}`;
}

function sampleExponential(ratePerSecond) {
  const u = Math.max(1e-9, 1 - Math.random());
  return -Math.log(u) / ratePerSecond;
}

function scheduleNextArrival(approachKey) {
  const ratePerHour = sim.config.arrivalRatePerHour[approachKey];
  const ratePerSecond = ratePerHour / 3600;
  if (ratePerSecond <= 0) {
    sim.nextArrivalTimes[approachKey] = Number.POSITIVE_INFINITY;
    return;
  }
  sim.nextArrivalTimes[approachKey] = sim.time + sampleExponential(ratePerSecond);
}

function resetSimulation() {
  sim.running = false;
  sim.lastTimestamp = 0;
  sim.time = 0;
  sim.vehicles = [];
  sim.nextVehicleId = 1;
  sim.stats = {
    arrivals: 0,
    departures: 0,
    currentQueue: 0,
    maxQueue: 0,
    totalDelay: 0,
  };
  sim.signal = {
    state: "ew-green",
    elapsed: 0,
  };
  sim.config = readConfig();
  sim.nextArrivalTimes = {};
  approachList.forEach((approach) => scheduleNextArrival(approach.key));
  updateOutputs();
  drawScene();
}

function pickMovement() {
  const leftTurnRatio = sim.config.leftTurnRatio;
  const remainingRatio = Math.max(0, 1 - leftTurnRatio);
  const nonLeftDefaultTotal = defaultMovementRatios.through + defaultMovementRatios.right;
  const throughRatio = nonLeftDefaultTotal > 0
    ? remainingRatio * (defaultMovementRatios.through / nonLeftDefaultTotal)
    : remainingRatio;
  const roll = Math.random();
  if (roll < leftTurnRatio) {
    return "left";
  }
  if (roll < leftTurnRatio + throughRatio) {
    return "through";
  }
  return "right";
}

function getVehiclePosition(approach, coord) {
  if (approach.axis === "x") {
    return point(coord, approach.laneCenter);
  }
  return point(approach.laneCenter, coord);
}

function createVehicle(approachKey) {
  const approach = approaches[approachKey];
  const movement = pickMovement();
  const coord = approach.spawnCoord;
  const desiredSpeed = sim.config.desiredSpeedMps * (0.88 + Math.random() * 0.24);

  // Determine if this is a large vehicle
  const isLargeVehicle = Math.random() < sim.config.largeVehicleRatio;
  const vehicleLength = isLargeVehicle ? 12.0 : 5.2;  // Large vehicles are ~12m, regular ~5.2m
  const vehicleWidth = isLargeVehicle ? 2.5 : 2.0;   // Large vehicles slightly wider

  return {
    id: sim.nextVehicleId++,
    approach: approachKey,
    movement,
    coord,
    v: desiredSpeed * 0.3,
    a: 0,
    length: vehicleLength,
    width: vehicleWidth,
    desiredSpeed,
    enteredAt: sim.time,
    progress: 0,
    routeKey: `${approachKey}-${movement}`,
    state: "approach",
    path: null,
    pathLength: 0,
    position: getVehiclePosition(approach, coord),
    angle: approach.pathDirection,
    color: isLargeVehicle
      ? `hsl(${220 + Math.random() * 20}, ${60 + Math.random() * 20}%, ${25 + Math.random() * 10}%)`  // Darker blue-gray for large vehicles
      : `hsl(${198 + Math.random() * 38}, ${54 + Math.random() * 18}%, ${38 + Math.random() * 15}%)`, // Original colors for regular vehicles
  };
}

function getApproachQueue(approachKey) {
  const approach = approaches[approachKey];
  return sim.vehicles
    .filter((vehicle) => vehicle.approach === approachKey && vehicle.state === "approach")
    .sort((a, b) => approach.queueOrder(a, b));
}

function maybeSpawnVehicles() {
  for (const approach of approachList) {
    while (sim.time >= sim.nextArrivalTimes[approach.key]) {
      const queue = getApproachQueue(approach.key);
      const lastVehicle = queue[queue.length - 1];
      const minSpawnGapPx = minVehicleGapPx + 8;
      const canSpawn = !lastVehicle || Math.abs(lastVehicle.coord - approach.spawnCoord) > minSpawnGapPx;

      if (!canSpawn) {
        sim.nextArrivalTimes[approach.key] += 0.5;
        break;
      }

      sim.vehicles.push(createVehicle(approach.key));
      sim.stats.arrivals += 1;
      scheduleNextArrival(approach.key);
    }
  }
}

function currentSignalDuration() {
  if (sim.signal.state.endsWith("yellow")) {
    return sim.config.yellow;
  }
  return sim.signal.state === "ew-green" ? sim.config.eastWestGreen : sim.config.northSouthGreen;
}

function updateSignal(dt) {
  sim.signal.elapsed += dt;
  if (sim.signal.elapsed < currentSignalDuration()) {
    return;
  }

  sim.signal.elapsed = 0;
  switch (sim.signal.state) {
    case "ew-green":
      sim.signal.state = "ew-yellow";
      break;
    case "ew-yellow":
      sim.signal.state = "ns-green";
      break;
    case "ns-green":
      sim.signal.state = "ns-yellow";
      break;
    default:
      sim.signal.state = "ew-green";
      break;
  }
}

function isVehiclePermitted(vehicle) {
  if (sim.signal.state === "ew-green") {
    return approaches[vehicle.approach].phaseGroup === "ew";
  }
  if (sim.signal.state === "ns-green") {
    return approaches[vehicle.approach].phaseGroup === "ns";
  }
  return false;
}

function getOpposingApproachKey(approachKey) {
  if (approachKey === "west") {
    return "east";
  }
  if (approachKey === "east") {
    return "west";
  }
  if (approachKey === "north") {
    return "south";
  }
  return "north";
}

function hasOpposingThroughConflict(vehicle) {
  if (vehicle.movement !== "left") {
    return false;
  }

  const conflictProfile = getLeftTurnConflictProfile(vehicle.approach);
  if (!conflictProfile) {
    return false;
  }

  const opposingKey = getOpposingApproachKey(vehicle.approach);
  const opposingLead = getApproachQueue(opposingKey)[0] || null;

  return sim.vehicles.some((other) => {
    if (other.id === vehicle.id || other.approach !== opposingKey || other.movement !== "through") {
      return false;
    }

    if (other.state === "approach") {
      if (!opposingLead || other.id !== opposingLead.id || !isVehiclePermitted(other)) {
        return false;
      }
    } else if (other.state !== "intersection") {
      return false;
    }

    const remainingDistance = getThroughDistanceToConflict(other, conflictProfile.throughConflictCoord);
    const clearanceDistance = (other.length * pxPerMeter) / 2 + 8;
    return remainingDistance >= -clearanceDistance && remainingDistance <= opposingThroughYieldDistancePx;
  });
}

function moveVehicleIntoIntersection(vehicle) {
  vehicle.state = "intersection";
  vehicle.path = buildPath(vehicle);
  vehicle.pathLength = pathLength(vehicle.path);
  vehicle.progress = 0;

  if (vehicle.movement !== "left") {
    return;
  }

  const conflictProfile = getLeftTurnConflictProfile(vehicle.approach);
  if (!conflictProfile) {
    return;
  }

  const halfVehicleLengthPx = (vehicle.length * pxPerMeter) / 2;
  vehicle.yieldProgress = Math.max(0, conflictProfile.yieldProgress - halfVehicleLengthPx);
  vehicle.clearProgress = Math.min(vehicle.pathLength, conflictProfile.clearProgress + halfVehicleLengthPx);
}

function buildPath(vehicle) {
  const approach = approaches[vehicle.approach];
  const stopPoint = getVehiclePosition(approach, approach.stopCoord + approach.travelSign * 3);
  const target = movementTargets[vehicle.approach][vehicle.movement];

  if (vehicle.movement === "through") {
    return [stopPoint, target];
  }
  const [control1, control2] = turnControlPoints[vehicle.approach][vehicle.movement];

  const points = [];
  const steps = vehicle.movement === "left" ? 18 : 12;
  for (let step = 0; step <= steps; step += 1) {
    const t = step / steps;
    const oneMinusT = 1 - t;
    points.push(point(
      oneMinusT ** 3 * stopPoint.x +
        3 * oneMinusT ** 2 * t * control1.x +
        3 * oneMinusT * t ** 2 * control2.x +
        t ** 3 * target.x,
      oneMinusT ** 3 * stopPoint.y +
        3 * oneMinusT ** 2 * t * control1.y +
        3 * oneMinusT * t ** 2 * control2.y +
        t ** 3 * target.y,
    ));
  }
  return points;
}

function pathLength(points) {
  let total = 0;
  for (let i = 1; i < points.length; i += 1) {
    total += distanceBetween(points[i - 1], points[i]);
  }
  return total;
}

function samplePath(points, distance) {
  let remaining = Math.max(0, distance);
  for (let i = 1; i < points.length; i += 1) {
    const start = points[i - 1];
    const end = points[i];
    const segmentLength = distanceBetween(start, end);
    if (remaining <= segmentLength || i === points.length - 1) {
      const ratio = segmentLength > 0 ? Math.min(1, remaining / segmentLength) : 1;
      return {
        position: point(
          start.x + (end.x - start.x) * ratio,
          start.y + (end.y - start.y) * ratio,
        ),
        angle: Math.atan2(end.y - start.y, end.x - start.x),
      };
    }
    remaining -= segmentLength;
  }

  const last = points[points.length - 1];
  const previous = points[points.length - 2];
  return {
    position: last,
    angle: Math.atan2(last.y - previous.y, last.x - previous.x),
  };
}

function findPathDistanceToLine(points, axis, value) {
  let cumulativeDistance = 0;

  for (let i = 1; i < points.length; i += 1) {
    const start = points[i - 1];
    const end = points[i];
    const startDelta = start[axis] - value;
    const endDelta = end[axis] - value;
    const segmentLength = distanceBetween(start, end);

    if (startDelta === 0) {
      return { distance: cumulativeDistance, point: start };
    }

    if (endDelta === 0) {
      return { distance: cumulativeDistance + segmentLength, point: end };
    }

    if ((startDelta < 0 && endDelta > 0) || (startDelta > 0 && endDelta < 0)) {
      const ratio = (value - start[axis]) / (end[axis] - start[axis]);
      return {
        distance: cumulativeDistance + segmentLength * ratio,
        point: point(
          start.x + (end.x - start.x) * ratio,
          start.y + (end.y - start.y) * ratio,
        ),
      };
    }

    cumulativeDistance += segmentLength;
  }

  return null;
}

const leftTurnConflictProfiles = {};

function getLeftTurnConflictProfile(approachKey) {
  if (Object.prototype.hasOwnProperty.call(leftTurnConflictProfiles, approachKey)) {
    return leftTurnConflictProfiles[approachKey];
  }

  const opposingKey = getOpposingApproachKey(approachKey);
  const opposingApproach = approaches[opposingKey];
  const conflictAxis = opposingApproach.axis === "x" ? "y" : "x";
  const leftTurnPath = buildPath({ approach: approachKey, movement: "left" });
  const conflictLocation = findPathDistanceToLine(leftTurnPath, conflictAxis, opposingApproach.laneCenter);

  if (!conflictLocation) {
    leftTurnConflictProfiles[approachKey] = null;
    return null;
  }

  const nominalPathLength = pathLength(leftTurnPath);
  leftTurnConflictProfiles[approachKey] = {
    throughConflictCoord: opposingApproach.axis === "x"
      ? conflictLocation.point.x
      : conflictLocation.point.y,
    yieldProgress: Math.max(0, conflictLocation.distance - leftTurnYieldBufferPx),
    clearProgress: Math.min(nominalPathLength, conflictLocation.distance + leftTurnYieldBufferPx),
  };

  return leftTurnConflictProfiles[approachKey];
}

function getVehicleAxisCoord(vehicle) {
  if (vehicle.state === "approach") {
    return vehicle.coord;
  }

  const approach = approaches[vehicle.approach];
  return approach.axis === "x" ? vehicle.position.x : vehicle.position.y;
}

function getThroughDistanceToConflict(vehicle, conflictCoord) {
  const approach = approaches[vehicle.approach];
  return (conflictCoord - getVehicleAxisCoord(vehicle)) * approach.travelSign;
}

function idmAcceleration(vehicle, leadObject) {
  const accel = 1.8;
  const decel = 2.6;
  const minGap = 2.5;
  const timeHeadway = 1.3;
  const delta = 4;

  const speed = Math.max(0, vehicle.v);
  const freeTerm = 1 - Math.pow(speed / Math.max(vehicle.desiredSpeed, 0.1), delta);

  if (!leadObject) {
    return accel * freeTerm;
  }

  const dv = speed - leadObject.speed;
  const desiredGap = minGap + Math.max(0, speed * timeHeadway + (speed * dv) / (2 * Math.sqrt(accel * decel)));
  const gap = Math.max(0.6, leadObject.gap);
  return accel * (freeTerm - Math.pow(desiredGap / gap, 2));
}

function updateApproachVehicles(dt) {
  for (const approach of approachList) {
    const queue = getApproachQueue(approach.key);

    for (let i = 0; i < queue.length; i += 1) {
      const vehicle = queue[i];
      const leader = i === 0 ? null : queue[i - 1];
      let leadObject = null;

      if (leader) {
        const gap = Math.abs(leader.coord - vehicle.coord) - leader.length * pxPerMeter;
        leadObject = {
          gap: Math.max(0.5, gap / pxPerMeter),
          speed: leader.v,
        };
      }

      if (!isVehiclePermitted(vehicle)) {
        const stopGap = Math.abs(approach.stopCoord - vehicle.coord) - (vehicle.length * pxPerMeter) / 2;
        const signalLeader = {
          gap: Math.max(0.5, stopGap / pxPerMeter),
          speed: 0,
        };
        if (!leadObject || signalLeader.gap < leadObject.gap) {
          leadObject = signalLeader;
        }
      }

      if (hasOpposingThroughConflict(vehicle)) {
        const yieldGap = Math.abs(approach.stopCoord - vehicle.coord) - (vehicle.length * pxPerMeter) / 2;
        const conflictLeader = {
          gap: Math.max(0.5, yieldGap / pxPerMeter),
          speed: 0,
        };
        if (!leadObject || conflictLeader.gap < leadObject.gap) {
          leadObject = conflictLeader;
        }
      }

      vehicle.a = idmAcceleration(vehicle, leadObject);
    }

    for (let i = 0; i < queue.length; i += 1) {
      const vehicle = queue[i];
      const leader = i === 0 ? null : queue[i - 1];
      vehicle.v = Math.max(0, vehicle.v + vehicle.a * dt);
      vehicle.coord += approach.travelSign * vehicle.v * pxPerMeter * dt;

      if (leader) {
        if (approach.travelSign > 0) {
          vehicle.coord = Math.min(vehicle.coord, leader.coord - minVehicleGapPx);
        } else {
          vehicle.coord = Math.max(vehicle.coord, leader.coord + minVehicleGapPx);
        }
      }

      vehicle.position = getVehiclePosition(approach, vehicle.coord);
      vehicle.angle = approach.pathDirection;

      const canEnter =
        isVehiclePermitted(vehicle) &&
        !hasOpposingThroughConflict(vehicle) &&
        (
          (approach.travelSign > 0 && vehicle.coord >= approach.stopCoord + 1) ||
          (approach.travelSign < 0 && vehicle.coord <= approach.stopCoord - 1)
        );

      if (canEnter) {
        moveVehicleIntoIntersection(vehicle);
      }
    }
  }
}

function updateIntersectionVehicles(dt) {
  const remaining = sim.vehicles.filter((vehicle) => vehicle.state !== "intersection");
  const active = sim.vehicles.filter((vehicle) => vehicle.state === "intersection");
  const routeGroups = new Map();

  for (const vehicle of active) {
    const shouldYieldInIntersection =
      vehicle.movement === "left" &&
      Number.isFinite(vehicle.yieldProgress) &&
      vehicle.progress <= vehicle.yieldProgress + 0.5 &&
      hasOpposingThroughConflict(vehicle);

    if (shouldYieldInIntersection) {
      vehicle.v = Math.max(0, vehicle.v - 3.6 * dt);
    } else {
      vehicle.v = Math.min(vehicle.desiredSpeed, Math.max(vehicle.desiredSpeed * 0.65, vehicle.v + 1.4 * dt));
    }

    const maxProgress = shouldYieldInIntersection ? vehicle.yieldProgress : Number.POSITIVE_INFINITY;
    vehicle.progress = Math.min(vehicle.progress + vehicle.v * pxPerMeter * dt, maxProgress);

    if (!routeGroups.has(vehicle.routeKey)) {
      routeGroups.set(vehicle.routeKey, []);
    }
    routeGroups.get(vehicle.routeKey).push(vehicle);
  }

  for (const group of routeGroups.values()) {
    group.sort((a, b) => b.progress - a.progress);
    for (let i = 1; i < group.length; i += 1) {
      const leader = group[i - 1];
      const follower = group[i];
      follower.progress = Math.min(follower.progress, leader.progress - minVehicleGapPx);
      follower.progress = Math.max(0, follower.progress);
      follower.v = Math.min(follower.v, leader.v);
    }
  }

  for (const vehicle of active) {
    const sample = samplePath(vehicle.path, vehicle.progress);
    vehicle.position = sample.position;
    vehicle.angle = sample.angle;

    if (vehicle.progress < vehicle.pathLength) {
      remaining.push(vehicle);
      continue;
    }

    const freeFlowTravelTime = 150 / vehicle.desiredSpeed;
    const elapsedTravelTime = sim.time - vehicle.enteredAt;
    const delay = Math.max(0, elapsedTravelTime - freeFlowTravelTime);
    sim.stats.departures += 1;
    sim.stats.totalDelay += delay;
  }

  sim.vehicles = remaining;
}

function updateVehicles(dt) {
  updateApproachVehicles(dt);
  updateIntersectionVehicles(dt);

  const queueCount = sim.vehicles.filter(
    (vehicle) => vehicle.state === "approach" && vehicle.v < 0.8,
  ).length;
  sim.stats.currentQueue = queueCount;
  sim.stats.maxQueue = Math.max(sim.stats.maxQueue, queueCount);
}

function formatSignalState() {
  if (sim.signal.state === "ew-green") {
    return "East-West Green";
  }
  if (sim.signal.state === "ns-green") {
    return "North-South Green";
  }
  if (sim.signal.state === "ew-yellow") {
    return "East-West Yellow";
  }
  return "North-South Yellow";
}

function updateOutputs() {
  sim.config = readConfig();
  controls.simSpeedLabel.textContent = `${sim.config.simSpeed.toFixed(1)}x`;
  statEls.arrivals.textContent = String(sim.stats.arrivals);
  statEls.departures.textContent = String(sim.stats.departures);
  statEls.queue.textContent = String(sim.stats.currentQueue);
  statEls.maxQueue.textContent = String(sim.stats.maxQueue);
  statEls.delay.textContent = `${(sim.stats.departures ? sim.stats.totalDelay / sim.stats.departures : 0).toFixed(1)} s`;
  statEls.signal.textContent = formatSignalState();
  statEls.status.textContent = sim.running ? "Running" : "Paused";
  statEls.clock.textContent = formatClock(sim.time);
}

function drawRoads() {
  const grassGradient = ctx.createLinearGradient(0, 0, 0, canvas.height);
  grassGradient.addColorStop(0, "#aad5a5");
  grassGradient.addColorStop(1, "#7fa47b");
  ctx.fillStyle = grassGradient;
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = "#4d555d";
  ctx.fillRect(0, roadCenterY - 76, canvas.width, 152);
  ctx.fillRect(roadCenterX - 76, 0, 152, canvas.height);

  ctx.fillStyle = "#59626c";
  ctx.fillRect(0, roadCenterY - 62, canvas.width, 124);
  ctx.fillRect(roadCenterX - 62, 0, 124, canvas.height);

  ctx.strokeStyle = "#facc15";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(0, roadCenterY - 5);
  ctx.lineTo(roadCenterX - 78, roadCenterY - 5);
  ctx.moveTo(roadCenterX + 78, roadCenterY - 5);
  ctx.lineTo(canvas.width, roadCenterY - 5);
  ctx.moveTo(0, roadCenterY + 5);
  ctx.lineTo(roadCenterX - 78, roadCenterY + 5);
  ctx.moveTo(roadCenterX + 78, roadCenterY + 5);
  ctx.lineTo(canvas.width, roadCenterY + 5);
  ctx.moveTo(roadCenterX - 5, 0);
  ctx.lineTo(roadCenterX - 5, roadCenterY - 78);
  ctx.moveTo(roadCenterX - 5, roadCenterY + 78);
  ctx.lineTo(roadCenterX - 5, canvas.height);
  ctx.moveTo(roadCenterX + 5, 0);
  ctx.lineTo(roadCenterX + 5, roadCenterY - 78);
  ctx.moveTo(roadCenterX + 5, roadCenterY + 78);
  ctx.lineTo(roadCenterX + 5, canvas.height);
  ctx.stroke();

  // Removed white dashed center lines
  // ctx.strokeStyle = "rgba(255,255,255,0.75)";
  // ctx.lineWidth = 3;
  // ctx.setLineDash([18, 18]);
  // ctx.beginPath();
  // ctx.moveTo(0, roadCenterY);
  // ctx.lineTo(roadCenterX - 78, roadCenterY);
  // ctx.moveTo(roadCenterX + 78, roadCenterY);
  // ctx.lineTo(canvas.width, roadCenterY);
  // ctx.moveTo(roadCenterX, 0);
  // ctx.lineTo(roadCenterX, roadCenterY - 78);
  // ctx.moveTo(roadCenterX, roadCenterY + 78);
  // ctx.lineTo(roadCenterX, canvas.height);
  // ctx.stroke();
  // ctx.setLineDash([]);

  ctx.fillStyle = "rgba(255,255,255,0.95)";
  ctx.fillRect(roadCenterX - roadHalfWidth - 10, roadCenterY + 2, 10, 44);
  ctx.fillRect(roadCenterX + roadHalfWidth, roadCenterY - 46, 10, 44);
  ctx.fillRect(roadCenterX - 46, roadCenterY - roadHalfWidth - 10, 44, 10);
  ctx.fillRect(roadCenterX + 2, roadCenterY + roadHalfWidth, 44, 10);
}

function signalColorForApproach(approachKey) {
  if (sim.signal.state === "ew-green" && approaches[approachKey].phaseGroup === "ew") {
    return "#22c55e";
  }
  if (sim.signal.state === "ns-green" && approaches[approachKey].phaseGroup === "ns") {
    return "#22c55e";
  }
  if (sim.signal.state === "ew-yellow" && approaches[approachKey].phaseGroup === "ew") {
    return "#fbbf24";
  }
  if (sim.signal.state === "ns-yellow" && approaches[approachKey].phaseGroup === "ns") {
    return "#fbbf24";
  }
  return "#4b1f24";
}

function drawSignalHeads() {
  for (const approach of approachList) {
    const color = signalColorForApproach(approach.key);
    ctx.fillStyle = "#1f2730";
    roundRect(ctx, approach.signalPos.x - 16, approach.signalPos.y - 24, 32, 48, 10);
    ctx.fill();

    ctx.beginPath();
    ctx.arc(approach.signalPos.x, approach.signalPos.y, 10, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
  }

  ctx.fillStyle = "rgba(255,255,255,0.92)";
  ctx.font = '700 16px "Space Grotesk", sans-serif';
  ctx.fillText(formatSignalState(), 30, 40);
  ctx.fillText(`Time Remaining: ${Math.max(0, currentSignalDuration() - sim.signal.elapsed).toFixed(1)} s`, 30, 64);
}

function drawVehicle(vehicle) {
  const lengthPx = vehicle.length * pxPerMeter;
  const widthPx = vehicle.width * pxPerMeter;
  const noseX = lengthPx / 2;
  const tailX = -lengthPx / 2;
  const wheelLength = Math.max(6, lengthPx * 0.18);
  const wheelWidth = Math.max(3, widthPx * 0.18);

  ctx.save();
  ctx.translate(vehicle.position.x, vehicle.position.y);
  ctx.rotate(vehicle.angle);

  ctx.shadowColor = "rgba(12, 24, 35, 0.28)";
  ctx.shadowBlur = 8;
  ctx.shadowOffsetY = 3;
  roundRect(ctx, tailX, -widthPx / 2, lengthPx, widthPx, 8);
  ctx.fillStyle = vehicle.color;
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.shadowOffsetY = 0;

  const bodyGradient = ctx.createLinearGradient(tailX, 0, noseX, 0);
  bodyGradient.addColorStop(0, "rgba(255,255,255,0.08)");
  bodyGradient.addColorStop(0.5, "rgba(255,255,255,0)");
  bodyGradient.addColorStop(1, "rgba(0,0,0,0.16)");
  roundRect(ctx, tailX + 1.5, -widthPx / 2 + 1.5, lengthPx - 3, widthPx - 3, 7);
  ctx.fillStyle = bodyGradient;
  ctx.fill();

  ctx.fillStyle = "rgba(24, 39, 64, 0.88)";
  roundRect(ctx, tailX + lengthPx * 0.2, -widthPx * 0.32, lengthPx * 0.48, widthPx * 0.64, 5);
  ctx.fill();

  ctx.fillStyle = "rgba(186, 220, 245, 0.72)";
  roundRect(ctx, tailX + lengthPx * 0.24, -widthPx * 0.26, lengthPx * 0.18, widthPx * 0.52, 4);
  ctx.fill();
  roundRect(ctx, tailX + lengthPx * 0.46, -widthPx * 0.26, lengthPx * 0.18, widthPx * 0.52, 4);
  ctx.fill();

  ctx.fillStyle = "#1f2937";
  roundRect(ctx, tailX + lengthPx * 0.16, -widthPx / 2 - 1, wheelLength, wheelWidth, 2);
  ctx.fill();
  roundRect(ctx, noseX - lengthPx * 0.16 - wheelLength, -widthPx / 2 - 1, wheelLength, wheelWidth, 2);
  ctx.fill();
  roundRect(ctx, tailX + lengthPx * 0.16, widthPx / 2 - wheelWidth + 1, wheelLength, wheelWidth, 2);
  ctx.fill();
  roundRect(ctx, noseX - lengthPx * 0.16 - wheelLength, widthPx / 2 - wheelWidth + 1, wheelLength, wheelWidth, 2);
  ctx.fill();

  ctx.fillStyle = "rgba(255, 248, 210, 0.95)";
  roundRect(ctx, noseX - 4, -widthPx * 0.24, 3, widthPx * 0.16, 1.5);
  ctx.fill();
  roundRect(ctx, noseX - 4, widthPx * 0.08, 3, widthPx * 0.16, 1.5);
  ctx.fill();

  ctx.fillStyle = "rgba(255, 92, 92, 0.95)";
  roundRect(ctx, tailX + 1, -widthPx * 0.24, 3, widthPx * 0.16, 1.5);
  ctx.fill();
  roundRect(ctx, tailX + 1, widthPx * 0.08, 3, widthPx * 0.16, 1.5);
  ctx.fill();

  ctx.restore();
}

function drawApproachLabels() {
  ctx.fillStyle = "rgba(255,255,255,0.92)";
  ctx.font = '700 14px "Space Grotesk", sans-serif';
  ctx.fillText("Eastbound", 54, roadCenterY + 102);
  ctx.fillText("Westbound", canvas.width - 146, roadCenterY - 86);
  ctx.fillText("Southbound", roadCenterX - 44, 24);
  ctx.fillText("Northbound", roadCenterX - 44, canvas.height - 14);
}

function roundRect(context, x, y, width, height, radius) {
  context.beginPath();
  context.moveTo(x + radius, y);
  context.arcTo(x + width, y, x + width, y + height, radius);
  context.arcTo(x + width, y + height, x, y + height, radius);
  context.arcTo(x, y + height, x, y, radius);
  context.arcTo(x, y, x + width, y, radius);
  context.closePath();
}

function drawOverlay() {
  ctx.fillStyle = "rgba(16, 44, 71, 0.8)";
  roundRect(ctx, 18, canvas.height - 118, 380, 88, 18);
  ctx.fill();

  ctx.fillStyle = "white";
  ctx.font = '700 18px "Space Grotesk", sans-serif';
  ctx.fillText("Intersection Performance", 36, canvas.height - 84);
  ctx.font = '600 15px "Source Sans 3", sans-serif';
  ctx.fillText(`Arrivals: ${sim.stats.arrivals}`, 36, canvas.height - 54);
  ctx.fillText(`Departures: ${sim.stats.departures}`, 150, canvas.height - 54);
  ctx.fillText(`Queue: ${sim.stats.currentQueue}`, 286, canvas.height - 54);
}

function drawScene() {
  drawRoads();
  drawSignalHeads();
  sim.vehicles.forEach(drawVehicle);
  drawApproachLabels();
  drawOverlay();
}

function step(timestamp) {
  if (!sim.lastTimestamp) {
    sim.lastTimestamp = timestamp;
  }

  const dtReal = Math.min(0.05, (timestamp - sim.lastTimestamp) / 1000);
  sim.lastTimestamp = timestamp;

  if (sim.running) {
    const dt = dtReal * sim.config.simSpeed;
    sim.time += dt;
    updateSignal(dt);
    maybeSpawnVehicles();
    updateVehicles(dt);
  }

  updateOutputs();
  drawScene();
  requestAnimationFrame(step);
}

controls.start.addEventListener("click", () => {
  sim.config = readConfig();
  sim.running = true;
  updateOutputs();
});

controls.pause.addEventListener("click", () => {
  sim.running = false;
  updateOutputs();
});

controls.reset.addEventListener("click", resetSimulation);

[
  controls.green,
  controls.yellow,
  controls.red,
  controls.arrivalRateWest,
  controls.arrivalRateEast,
  controls.arrivalRateNorth,
  controls.arrivalRateSouth,
  controls.desiredSpeed,
  controls.leftTurnRatio,
  controls.largeVehicleRatio,
  controls.simSpeed,
].forEach((control) => {
  control.addEventListener("input", () => {
    updateOutputs();
    if (
      control === controls.arrivalRateWest ||
      control === controls.arrivalRateEast ||
      control === controls.arrivalRateNorth ||
      control === controls.arrivalRateSouth
    ) {
      approachList.forEach((approach) => scheduleNextArrival(approach.key));
    }
  });
});

resetSimulation();
requestAnimationFrame(step);

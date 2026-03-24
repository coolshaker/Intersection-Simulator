const canvas = document.getElementById("sim-canvas");
const ctx = canvas.getContext("2d");

const controls = {
  green: document.getElementById("green-duration"),
  yellow: document.getElementById("yellow-duration"),
  allRed: document.getElementById("all-red-duration"),
  red: document.getElementById("red-duration"),
  arrivalRateWest: document.getElementById("arrival-rate-west"),
  arrivalRateEast: document.getElementById("arrival-rate-east"),
  arrivalRateNorth: document.getElementById("arrival-rate-north"),
  arrivalRateSouth: document.getElementById("arrival-rate-south"),
  pedVolumeTotal: document.getElementById("ped-volume-total"),
  desiredSpeed: document.getElementById("desired-speed"),
  leftTurnRatio: document.getElementById("left-turn-ratio"),
  rightTurnRatio: document.getElementById("right-turn-ratio"),
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
  avgQueue: document.getElementById("avg-queue-stat"),
  maxQueue: document.getElementById("max-queue-stat"),
  delay: document.getElementById("delay-stat"),
  status: document.getElementById("sim-status"),
  clock: document.getElementById("clock-display"),
};

const pxPerMeter = 5.2;
const roadCenterX = 520;
const roadCenterY = canvas.height / 2;
const roadHalfWidth = 60;
const approachLength = 420;
const exitLength = 220;
const minVehicleGapPx = 12.0 * pxPerMeter + 8;  // Based on max vehicle length (12m) + buffer
const intersectionEntryAwarenessDistancePx = minVehicleGapPx + 12;
const opposingThroughYieldDistancePx = 190;
const opposingRightTurnYieldDistancePx = 130;
const queueDetectionDistancePx = 14 * pxPerMeter;
const queueCreepSpeedMps = 2.2;
const queueFollowerGapPx = minVehicleGapPx + 10;
const leftTurnYieldBufferPx = 18;
const sharedLaneConflictThresholdPx = 12;
const crosswalkHalfSpanPx = 60;
const crosswalkWidthPx = 36;
const crosswalkInnerSetbackPx = 19;
const crosswalkOffsetPx = crosswalkInnerSetbackPx + crosswalkWidthPx / 2;
const stopLineGapFromCrosswalkPx = 12;
const stopLineOffsetPx = crosswalkOffsetPx + crosswalkWidthPx / 2 + stopLineGapFromCrosswalkPx;
const stopLineThicknessPx = 6;
const stopLineLengthPx = 52;
const centerlineEndClearancePx = stopLineThicknessPx / 2 + 18;
const centerlineSetbackPx = roadHalfWidth + stopLineOffsetPx + centerlineEndClearancePx;
const crosswalkStripeThicknessPx = 8;
const pedBodyRadiusPx = 4.5;
const pedestrianPathConflictThresholdPx = 16;
const pedestrianYieldBufferPx = 12;
const pedestrianStartDelayMinS = 0.25;
const pedestrianStartDelayMaxS = 1.1;
const pedestrianAccelerationMps2 = 0.9;
const pedestrianEndSlowdownDistancePx = 18;
const pedestrianComfortGapPx = 26;
const pedestrianLeadVehicleSpeedThresholdMps = 1.8;
const pedestrianLateralOffsetPx = 4.5;
const pedestrianLaneOffsetsPx = [-9, 0, 9];
const pedestrianSpawnSpacingPx = 24;
const pedestrianSwayAmplitudePx = 1.3;
const pedestrianVehicleClearancePx = 2.5;
const pedestrianMaxDodgeOffsetPx = 8;
const pedestrianStoppedVehicleDodgeOffsetPx = 16;
const pedestrianDodgeRatePxPerSecond = 28;
const pedestrianStoppedVehicleSpeedThresholdMps = 0.15;
const pedestrianLargeVehicleBypassMarginPx = 6;
function point(x, y) {
  return { x, y };
}

function distanceBetween(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function getFollowingGapPx(approach, leaderCoord, leaderLength, followerCoord, followerLength) {
  const leaderRearCoord = leaderCoord - approach.travelSign * (leaderLength * pxPerMeter) / 2;
  const followerFrontCoord = followerCoord + approach.travelSign * (followerLength * pxPerMeter) / 2;
  return (leaderRearCoord - followerFrontCoord) * approach.travelSign;
}

function distancePointToSegment(target, start, end) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const lengthSquared = dx * dx + dy * dy;

  if (lengthSquared === 0) {
    return {
      distance: distanceBetween(target, start),
      point: start,
      ratio: 0,
    };
  }

  const projection = ((target.x - start.x) * dx + (target.y - start.y) * dy) / lengthSquared;
  const ratio = clamp(projection, 0, 1);
  const pointOnSegment = point(start.x + dx * ratio, start.y + dy * ratio);

  return {
    distance: distanceBetween(target, pointOnSegment),
    point: pointOnSegment,
    ratio,
  };
}

function findClosestPathPoint(points, target) {
  if (!points || points.length < 2) {
    return null;
  }

  let cumulativeDistance = 0;
  let best = null;

  for (let i = 1; i < points.length; i += 1) {
    const start = points[i - 1];
    const end = points[i];
    const segmentLength = distanceBetween(start, end);
    const closest = distancePointToSegment(target, start, end);
    const progress = cumulativeDistance + segmentLength * closest.ratio;

    if (!best || closest.distance < best.distance) {
      best = {
        distance: closest.distance,
        point: closest.point,
        progress,
      };
    }

    cumulativeDistance += segmentLength;
  }

  return best;
}

function normalizeVector(dx, dy) {
  const magnitude = Math.hypot(dx, dy);
  if (magnitude < 1e-6) {
    return { x: 0, y: 0 };
  }
  return { x: dx / magnitude, y: dy / magnitude };
}

function moveToward(current, target, maxDelta) {
  if (Math.abs(target - current) <= maxDelta) {
    return target;
  }
  return current + Math.sign(target - current) * maxDelta;
}

function isVehicleStoppedForPedestrian(vehicle) {
  return Math.abs(vehicle.v) <= pedestrianStoppedVehicleSpeedThresholdMps;
}

const approaches = {
  west: {
    key: "west",
    label: "Eastbound",
    axis: "x",
    phaseGroup: "ew",
    spawnCoord: -approachLength,
    stopCoord: roadCenterX - roadHalfWidth - stopLineOffsetPx,
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
    stopCoord: roadCenterX + roadHalfWidth + stopLineOffsetPx,
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
    stopCoord: roadCenterY - roadHalfWidth - stopLineOffsetPx,
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
    stopCoord: roadCenterY + roadHalfWidth + stopLineOffsetPx,
    travelSign: -1,
    queueOrder: (a, b) => a.coord - b.coord,
    pathDirection: -Math.PI / 2,
    laneCenter: roadCenterX + 30,
    signalPos: point(roadCenterX + 92, roadCenterY + 90),
  },
};

const approachList = Object.values(approaches);
const crosswalks = {
  west: {
    start: point(roadCenterX - roadHalfWidth - crosswalkOffsetPx, roadCenterY - crosswalkHalfSpanPx),
    end: point(roadCenterX - roadHalfWidth - crosswalkOffsetPx, roadCenterY + crosswalkHalfSpanPx),
  },
  east: {
    start: point(roadCenterX + roadHalfWidth + crosswalkOffsetPx, roadCenterY + crosswalkHalfSpanPx),
    end: point(roadCenterX + roadHalfWidth + crosswalkOffsetPx, roadCenterY - crosswalkHalfSpanPx),
  },
  north: {
    start: point(roadCenterX + crosswalkHalfSpanPx, roadCenterY - roadHalfWidth - crosswalkOffsetPx),
    end: point(roadCenterX - crosswalkHalfSpanPx, roadCenterY - roadHalfWidth - crosswalkOffsetPx),
  },
  south: {
    start: point(roadCenterX - crosswalkHalfSpanPx, roadCenterY + roadHalfWidth + crosswalkOffsetPx),
    end: point(roadCenterX + crosswalkHalfSpanPx, roadCenterY + roadHalfWidth + crosswalkOffsetPx),
  },
};

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
  pedestrians: [],
  nextVehicleId: 1,
  nextPedestrianId: 1,
  nextArrivalTimes: {},
  nextPedArrivalTimes: {},
  pendingPedRequests: {},
  stats: {
    arrivals: 0,
    departures: 0,
    completedTrips: 0,
    currentQueue: 0,
    queueTimeIntegral: 0,
    maxQueue: 0,
    totalDelay: 0,
  },
  signal: {
    state: "ew-green",
    elapsed: 0,
    yellowLeftTurnClearance: {},
  },
  config: readConfig(),
};

function readConfig() {
  const leftTurnRatio = clampNumber(controls.leftTurnRatio.value, 0, 100, 5) / 100;
  const requestedRightTurnRatio = clampNumber(controls.rightTurnRatio.value, 0, 100, 5) / 100;
  const rightTurnRatio = Math.min(requestedRightTurnRatio, 1 - leftTurnRatio);

  return {
    eastWestGreen: clampNumber(controls.green.value, 5, 120, 18),
    yellow: clampNumber(controls.yellow.value, 2, 20, 4),
    allRed: clampNumber(controls.allRed.value, 0.5, 10, 2),
    northSouthGreen: clampNumber(controls.red.value, 5, 120, 20),
    arrivalRatePerHour: {
      west: clampNumber(controls.arrivalRateWest.value, 0, 1800, 300),
      east: clampNumber(controls.arrivalRateEast.value, 0, 1800, 300),
      north: clampNumber(controls.arrivalRateNorth.value, 0, 1800, 300),
      south: clampNumber(controls.arrivalRateSouth.value, 0, 1800, 300),
    },
    pedestrianTotalRatePerHour: clampNumber(controls.pedVolumeTotal.value, 0, 12000, 100),
    desiredSpeedMps: clampNumber(controls.desiredSpeed.value, 10, 45, 28) * 0.44704,
    leftTurnRatio,
    rightTurnRatio,
    largeVehicleRatio: clampNumber(controls.largeVehicleRatio.value, 0, 100, 5) / 100,
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

function scheduleNextPedArrival(approachKey) {
  const ratePerHour = sim.config.pedestrianTotalRatePerHour / approachList.length;
  const ratePerSecond = ratePerHour / 3600;
  if (ratePerSecond <= 0) {
    sim.nextPedArrivalTimes[approachKey] = Number.POSITIVE_INFINITY;
    return;
  }
  sim.nextPedArrivalTimes[approachKey] = sim.time + sampleExponential(ratePerSecond);
}

function resetSimulation() {
  sim.running = false;
  sim.lastTimestamp = 0;
  sim.time = 0;
  sim.vehicles = [];
  sim.pedestrians = [];
  sim.nextVehicleId = 1;
  sim.nextPedestrianId = 1;
  sim.stats = {
    arrivals: 0,
    departures: 0,
    completedTrips: 0,
    currentQueue: 0,
    queueTimeIntegral: 0,
    maxQueue: 0,
    totalDelay: 0,
  };
  sim.signal = {
    state: "ew-green",
    elapsed: 0,
    yellowLeftTurnClearance: {},
  };
  sim.config = readConfig();
  sim.nextArrivalTimes = {};
  sim.nextPedArrivalTimes = {};
  sim.pendingPedRequests = {};
  approachList.forEach((approach) => scheduleNextArrival(approach.key));
  approachList.forEach((approach) => {
    scheduleNextPedArrival(approach.key);
    sim.pendingPedRequests[approach.key] = 0;
  });
  updateOutputs();
  drawScene();
}

function pickMovement() {
  const leftTurnRatio = sim.config.leftTurnRatio;
  const rightTurnRatio = Math.min(sim.config.rightTurnRatio, Math.max(0, 1 - leftTurnRatio));
  const throughRatio = Math.max(0, 1 - leftTurnRatio - rightTurnRatio);
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
    hasEnteredView: false,
    hasExitedView: false,
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
      scheduleNextArrival(approach.key);
    }
  }
}

function getVehicleCanvasHalfExtents(vehicle) {
  const halfLengthPx = (vehicle.length * pxPerMeter) / 2;
  const halfWidthPx = (vehicle.width * pxPerMeter) / 2;

  return {
    x: Math.abs(Math.cos(vehicle.angle)) * halfLengthPx + Math.abs(Math.sin(vehicle.angle)) * halfWidthPx,
    y: Math.abs(Math.sin(vehicle.angle)) * halfLengthPx + Math.abs(Math.cos(vehicle.angle)) * halfWidthPx,
  };
}

function isVehicleVisibleInView(vehicle) {
  const halfExtents = getVehicleCanvasHalfExtents(vehicle);

  return (
    vehicle.position.x + halfExtents.x >= 0 &&
    vehicle.position.x - halfExtents.x <= canvas.width &&
    vehicle.position.y + halfExtents.y >= 0 &&
    vehicle.position.y - halfExtents.y <= canvas.height
  );
}

function updateVehicleViewCounts(vehicle) {
  const isVisible = isVehicleVisibleInView(vehicle);

  if (isVisible && !vehicle.hasEnteredView) {
    vehicle.hasEnteredView = true;
    sim.stats.arrivals += 1;
  }

  if (!isVisible && vehicle.hasEnteredView && !vehicle.hasExitedView) {
    vehicle.hasExitedView = true;
    sim.stats.departures += 1;
  }
}

function getApproachQueueCount(approachKey) {
  const approach = approaches[approachKey];
  const queue = getApproachQueue(approachKey);
  const queuedVehicleIds = new Set();
  let count = 0;

  for (let i = 0; i < queue.length; i += 1) {
    const vehicle = queue[i];
    const leader = i === 0 ? null : queue[i - 1];
    const gapToStopPx = Math.abs(approach.stopCoord - vehicle.coord) - (vehicle.length * pxPerMeter) / 2;
    const gapToLeaderPx = leader
      ? Math.abs(leader.coord - vehicle.coord) - leader.length * pxPerMeter
      : Number.POSITIVE_INFINITY;
    const isStoppedAtControl =
      gapToStopPx <= queueDetectionDistancePx &&
      vehicle.v <= queueCreepSpeedMps;
    const isTrappedBehindQueue =
      Boolean(leader) &&
      queuedVehicleIds.has(leader.id) &&
      gapToLeaderPx <= queueFollowerGapPx;

    if (isStoppedAtControl || isTrappedBehindQueue) {
      queuedVehicleIds.add(vehicle.id);
      count += 1;
    }
  }

  return count;
}

function getTotalRealTimeQueueCount() {
  return approachList.reduce((total, approach) => total + getApproachQueueCount(approach.key), 0);
}

function currentSignalDuration() {
  if (sim.signal.state.endsWith("all-red")) {
    return sim.config.allRed;
  }
  if (sim.signal.state.endsWith("yellow")) {
    return sim.config.yellow;
  }
  return sim.signal.state === "ew-green" ? sim.config.eastWestGreen : sim.config.northSouthGreen;
}

function clearYellowLeftTurnClearance() {
  sim.signal.yellowLeftTurnClearance = {};
}

function captureYellowLeftTurnClearance(phaseGroup) {
  clearYellowLeftTurnClearance();

  for (const approach of approachList) {
    if (approach.phaseGroup !== phaseGroup) {
      continue;
    }

    const leadVehicle = getApproachQueue(approach.key)[0] || null;
    if (leadVehicle && leadVehicle.movement === "left") {
      sim.signal.yellowLeftTurnClearance[approach.key] = leadVehicle.id;
    }
  }
}

function isApproachVehicleGreen(approachKey) {
  if (sim.signal.state === "ew-green") {
    return approaches[approachKey].phaseGroup === "ew";
  }
  if (sim.signal.state === "ns-green") {
    return approaches[approachKey].phaseGroup === "ns";
  }
  return false;
}

function updateSignal(dt) {
  sim.signal.elapsed += dt;
  if (sim.signal.elapsed < currentSignalDuration()) {
    return;
  }

  sim.signal.elapsed = 0;
  switch (sim.signal.state) {
    case "ew-green":
      captureYellowLeftTurnClearance("ew");
      sim.signal.state = "ew-yellow";
      break;
    case "ew-yellow":
      clearYellowLeftTurnClearance();
      sim.signal.state = "ew-all-red";
      break;
    case "ew-all-red":
      clearYellowLeftTurnClearance();
      sim.signal.state = "ns-green";
      break;
    case "ns-green":
      captureYellowLeftTurnClearance("ns");
      sim.signal.state = "ns-yellow";
      break;
    case "ns-yellow":
      clearYellowLeftTurnClearance();
      sim.signal.state = "ns-all-red";
      break;
    default:
      clearYellowLeftTurnClearance();
      sim.signal.state = "ew-green";
      break;
  }
}

function isVehiclePermitted(vehicle) {
  if (isApproachVehicleGreen(vehicle.approach)) {
    return true;
  }

  if (!sim.signal.state.endsWith("yellow")) {
    return false;
  }

  return vehicle.movement === "left" &&
    approaches[vehicle.approach].phaseGroup === sim.signal.state.slice(0, 2) &&
    sim.signal.yellowLeftTurnClearance[vehicle.approach] === vehicle.id;
}

function hasActiveCrosswalkPedestrian(approachKey) {
  return sim.pedestrians.some((pedestrian) => pedestrian.approach === approachKey);
}

function getCrosswalkPedestrians(approachKey) {
  return sim.pedestrians.filter((pedestrian) => pedestrian.approach === approachKey);
}

function isPedestrianEntryComfortable(approachKey) {
  const queue = getApproachQueue(approachKey);
  const leadVehicle = queue[0];
  if (!leadVehicle) {
    return true;
  }

  const approach = approaches[approachKey];
  const gapToStopPx = Math.abs(approach.stopCoord - leadVehicle.coord) - (leadVehicle.length * pxPerMeter) / 2;
  if (gapToStopPx > pedestrianComfortGapPx) {
    return true;
  }

  return !isVehiclePermitted(leadVehicle) && leadVehicle.v <= pedestrianLeadVehicleSpeedThresholdMps;
}

function getAvailablePedestrianLaneOffset(approachKey) {
  const crosswalkPedestrians = getCrosswalkPedestrians(approachKey);

  for (const laneOffsetPx of pedestrianLaneOffsetsPx) {
    const laneOccupiedNearEntry = crosswalkPedestrians.some((pedestrian) => (
      Math.abs((pedestrian.laneOffsetPx ?? pedestrian.lateralOffsetPx ?? 0) - laneOffsetPx) < 2 &&
      pedestrian.progress < pedestrianSpawnSpacingPx
    ));

    if (!laneOccupiedNearEntry) {
      return laneOffsetPx;
    }
  }

  return null;
}

function createPedestrian(approachKey, laneOffsetPx = 0) {
  const crosswalk = crosswalks[approachKey];
  const direction = normalizeVector(crosswalk.end.x - crosswalk.start.x, crosswalk.end.y - crosswalk.start.y);
  const normal = point(-direction.y, direction.x);
  const baseOffsetPx = laneOffsetPx + (Math.random() * 2 - 1) * pedestrianLateralOffsetPx * 0.35;
  const start = point(
    crosswalk.start.x + normal.x * baseOffsetPx,
    crosswalk.start.y + normal.y * baseOffsetPx
  );
  const end = point(
    crosswalk.end.x + normal.x * baseOffsetPx,
    crosswalk.end.y + normal.y * baseOffsetPx
  );
  const lengthPx = distanceBetween(start, end);
  const desiredSpeedMps = 0.95 + Math.random() * 0.65;
  const bodyRadiusPx = pedBodyRadiusPx * (0.9 + Math.random() * 0.22);
  const stepFrequencyHz = 1.45 + desiredSpeedMps * 0.55 + Math.random() * 0.35;
  const swayAmplitudePx = pedestrianSwayAmplitudePx * (0.7 + Math.random() * 0.7);
  const startDelayS = pedestrianStartDelayMinS + Math.random() * (pedestrianStartDelayMaxS - pedestrianStartDelayMinS);
  return {
    id: sim.nextPedestrianId++,
    approach: approachKey,
    progress: 0,
    speedMps: 0,
    desiredSpeedMps,
    bodyRadiusPx,
    stepFrequencyHz,
    stepPhase: Math.random() * Math.PI * 2,
    swayAmplitudePx,
    laneOffsetPx,
    lateralOffsetPx: baseOffsetPx,
    startDelayS,
    start,
    end,
    lengthPx,
    position: start,
    heading: direction,
    normal,
    dodgeOffsetPx: 0,
    dodgeTargetOffsetPx: 0,
    blockingVehicleId: null,
  };
}

function maybeSpawnPedestrians() {
  for (const approach of approachList) {
    while (sim.time >= sim.nextPedArrivalTimes[approach.key]) {
      sim.pendingPedRequests[approach.key] += 1;
      scheduleNextPedArrival(approach.key);
    }

    const canStartCrossing = !isApproachVehicleGreen(approach.key) && isPedestrianEntryComfortable(approach.key);
    while (sim.pendingPedRequests[approach.key] > 0 && canStartCrossing) {
      const laneOffsetPx = getAvailablePedestrianLaneOffset(approach.key);
      if (laneOffsetPx === null) {
        break;
      }
      sim.pedestrians.push(createPedestrian(approach.key, laneOffsetPx));
      sim.pendingPedRequests[approach.key] -= 1;
    }
  }
}

function getPedestrianPositionAtProgress(pedestrian, progress, dodgeOffsetPx = pedestrian.dodgeOffsetPx || 0) {
  const ratio = Math.min(1, Math.max(0, progress / Math.max(1, pedestrian.lengthPx)));
  return point(
    pedestrian.start.x + (pedestrian.end.x - pedestrian.start.x) * ratio + pedestrian.normal.x * dodgeOffsetPx,
    pedestrian.start.y + (pedestrian.end.y - pedestrian.start.y) * ratio + pedestrian.normal.y * dodgeOffsetPx
  );
}

function getBlockingVehicleForPedestrian(pedestrian, position) {
  const bodyRadius = pedestrian.bodyRadiusPx || pedBodyRadiusPx;

  for (const vehicle of sim.vehicles) {
    const dx = position.x - vehicle.position.x;
    const dy = position.y - vehicle.position.y;
    const cos = Math.cos(vehicle.angle);
    const sin = Math.sin(vehicle.angle);
    const localX = dx * cos + dy * sin;
    const localY = -dx * sin + dy * cos;
    const halfLengthPx = (vehicle.length * pxPerMeter) / 2 + bodyRadius + pedestrianVehicleClearancePx;
    const halfWidthPx = (vehicle.width * pxPerMeter) / 2 + bodyRadius + pedestrianVehicleClearancePx;

    if (Math.abs(localX) <= halfLengthPx && Math.abs(localY) <= halfWidthPx) {
      return vehicle;
    }
  }

  return null;
}

function getPedestrianDodgeCandidates(pedestrian, blocker) {
  const bodyRadius = pedestrian.bodyRadiusPx || pedBodyRadiusPx;
  const side = ((blocker.position.x - pedestrian.position.x) * pedestrian.normal.x) +
    ((blocker.position.y - pedestrian.position.y) * pedestrian.normal.y);
  const preferredSign = side >= 0 ? -1 : 1;
  const maxDodgeOffset = isVehicleStoppedForPedestrian(blocker)
    ? pedestrianStoppedVehicleDodgeOffsetPx
    : pedestrianMaxDodgeOffsetPx;
  const blockerHalfExtents = getVehicleCanvasHalfExtents(blocker);
  const blockerNormalExtentPx = Math.abs(pedestrian.normal.x) > Math.abs(pedestrian.normal.y)
    ? blockerHalfExtents.x
    : blockerHalfExtents.y;
  const basePathPosition = getPedestrianPositionAtProgress(pedestrian, pedestrian.progress, 0);
  const blockerCenterOffsetPx =
    ((blocker.position.x - basePathPosition.x) * pedestrian.normal.x) +
    ((blocker.position.y - basePathPosition.y) * pedestrian.normal.y);
  const requiredBypassOffsetPx =
    Math.abs(blockerCenterOffsetPx) +
    blockerNormalExtentPx +
    bodyRadius +
    pedestrianVehicleClearancePx +
    pedestrianLargeVehicleBypassMarginPx;
  const stoppedVehicleBypassOffsetPx = isVehicleStoppedForPedestrian(blocker)
    ? Math.max(maxDodgeOffset, requiredBypassOffsetPx)
    : maxDodgeOffset;
  const offsets = [
    0,
    pedestrian.dodgeOffsetPx || 0,
    preferredSign * (maxDodgeOffset * 0.5),
    preferredSign * maxDodgeOffset,
    -preferredSign * (maxDodgeOffset * 0.5),
    -preferredSign * maxDodgeOffset,
    preferredSign * stoppedVehicleBypassOffsetPx,
    -preferredSign * stoppedVehicleBypassOffsetPx,
  ];

  return offsets
    .map((offset) => clamp(offset, -stoppedVehicleBypassOffsetPx, stoppedVehicleBypassOffsetPx))
    .filter((offset, index, array) => array.findIndex((value) => Math.abs(value - offset) < 0.01) === index);
}

function updatePedestrians(dt) {
  const remaining = [];
  for (const pedestrian of sim.pedestrians) {
    if (pedestrian.startDelayS > 0) {
      pedestrian.startDelayS = Math.max(0, pedestrian.startDelayS - dt);
    }

    let targetSpeedMps = pedestrian.startDelayS > 0 ? 0 : pedestrian.desiredSpeedMps;
    const remainingDistancePx = Math.max(0, pedestrian.lengthPx - pedestrian.progress);
    if (remainingDistancePx < pedestrianEndSlowdownDistancePx) {
      const slowdownRatio = clamp(remainingDistancePx / pedestrianEndSlowdownDistancePx, 0.35, 1);
      targetSpeedMps *= slowdownRatio;
    }

    const speedDeltaLimit = pedestrianAccelerationMps2 * dt;
    if (pedestrian.speedMps < targetSpeedMps) {
      pedestrian.speedMps = Math.min(targetSpeedMps, pedestrian.speedMps + speedDeltaLimit);
    } else {
      pedestrian.speedMps = Math.max(targetSpeedMps, pedestrian.speedMps - speedDeltaLimit * 1.2);
    }

    const currentProgress = pedestrian.progress;
    const currentPosition = getPedestrianPositionAtProgress(pedestrian, currentProgress);
    const desiredProgress = Math.min(
      pedestrian.lengthPx,
      currentProgress + pedestrian.speedMps * pxPerMeter * dt
    );

    let chosenProgress = desiredProgress;
    let chosenDodgeTarget = 0;
    let blocker = getBlockingVehicleForPedestrian(
      pedestrian,
      getPedestrianPositionAtProgress(pedestrian, desiredProgress, 0)
    );

    if (blocker) {
      const dodgeCandidates = getPedestrianDodgeCandidates(pedestrian, blocker);
      const forwardDodge = dodgeCandidates.find((offset) => (
        !getBlockingVehicleForPedestrian(
          pedestrian,
          getPedestrianPositionAtProgress(pedestrian, desiredProgress, offset)
        )
      ));

      if (typeof forwardDodge === "number") {
        chosenDodgeTarget = forwardDodge;
      } else {
        chosenProgress = currentProgress;
        const lateralDodge = dodgeCandidates.find((offset) => (
          !getBlockingVehicleForPedestrian(
            pedestrian,
            getPedestrianPositionAtProgress(pedestrian, currentProgress, offset)
          )
        ));
        if (typeof lateralDodge === "number") {
          chosenDodgeTarget = lateralDodge;
        } else {
          chosenDodgeTarget = pedestrian.dodgeOffsetPx || 0;
          pedestrian.speedMps = 0;
        }
      }
    }
    pedestrian.blockingVehicleId = blocker ? blocker.id : null;

    const relaxedCenterlinePosition = getPedestrianPositionAtProgress(pedestrian, desiredProgress, 0);
    if (!blocker && !getBlockingVehicleForPedestrian(pedestrian, relaxedCenterlinePosition)) {
      chosenDodgeTarget = 0;
    }

    pedestrian.dodgeTargetOffsetPx = chosenDodgeTarget;
    let nextDodgeOffset;
    if (blocker && isVehicleStoppedForPedestrian(blocker)) {
      nextDodgeOffset = pedestrian.dodgeTargetOffsetPx;
    } else if (
      chosenProgress === currentProgress &&
      Math.abs((pedestrian.dodgeOffsetPx || 0) - pedestrian.dodgeTargetOffsetPx) > 0.01
    ) {
      nextDodgeOffset = pedestrian.dodgeTargetOffsetPx;
    } else {
      nextDodgeOffset = moveToward(
        pedestrian.dodgeOffsetPx || 0,
        pedestrian.dodgeTargetOffsetPx,
        pedestrianDodgeRatePxPerSecond * dt
      );
    }
    let nextPosition = getPedestrianPositionAtProgress(pedestrian, chosenProgress, nextDodgeOffset);
    const nextBlocker = getBlockingVehicleForPedestrian(pedestrian, nextPosition);
    if (nextBlocker) {
      chosenProgress = currentProgress;
      nextPosition = currentPosition;
      pedestrian.speedMps = 0;
    } else {
      pedestrian.dodgeOffsetPx = nextDodgeOffset;
      pedestrian.progress = chosenProgress;
    }

    pedestrian.position = nextPosition;
    const ratio = Math.min(1, pedestrian.progress / Math.max(1, pedestrian.lengthPx));
    if (ratio < 1) {
      remaining.push(pedestrian);
    }
  }
  sim.pedestrians = remaining;
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

function hasOpposingRightTurnConflict(vehicle) {
  if (vehicle.movement !== "left") {
    return false;
  }

  const conflictProfile = getLeftTurnOpposingRightConflictProfile(vehicle.approach);
  if (!conflictProfile) {
    return false;
  }

  const opposingKey = getOpposingApproachKey(vehicle.approach);
  const opposingApproach = approaches[opposingKey];
  const opposingLead = getApproachQueue(opposingKey)[0] || null;

  return sim.vehicles.some((other) => {
    if (other.id === vehicle.id || other.approach !== opposingKey || other.movement !== "right") {
      return false;
    }

    if (other.state === "approach") {
      if (!opposingLead || other.id !== opposingLead.id || !isVehiclePermitted(other)) {
        return false;
      }

      const gapToStopPx = Math.abs(opposingApproach.stopCoord - other.coord) - (other.length * pxPerMeter) / 2;
      return gapToStopPx <= opposingRightTurnYieldDistancePx;
    }

    if (other.state !== "intersection") {
      return false;
    }

    const otherClearProgress = conflictProfile.opposingClearProgress + (other.length * pxPerMeter) / 2;
    return other.progress <= otherClearProgress;
  });
}

function moveVehicleIntoIntersection(vehicle) {
  vehicle.state = "intersection";
  vehicle.path = buildPath(vehicle);
  vehicle.pathLength = pathLength(vehicle.path);
  vehicle.progress = 0;

  vehicle.yieldProgress = Number.NaN;
  vehicle.clearProgress = Number.NaN;
  vehicle.opposingRightTurnYieldProgress = Number.NaN;
  vehicle.opposingRightTurnClearProgress = Number.NaN;

  if (vehicle.movement !== "left") {
    return;
  }

  const throughConflictProfile = getLeftTurnConflictProfile(vehicle.approach);
  const opposingRightConflictProfile = getLeftTurnOpposingRightConflictProfile(vehicle.approach);
  const halfVehicleLengthPx = (vehicle.length * pxPerMeter) / 2;

  if (throughConflictProfile) {
    vehicle.yieldProgress = Math.max(0, throughConflictProfile.yieldProgress - halfVehicleLengthPx);
    vehicle.clearProgress = Math.min(vehicle.pathLength, throughConflictProfile.clearProgress + halfVehicleLengthPx);
  }

  if (opposingRightConflictProfile) {
    vehicle.opposingRightTurnYieldProgress = Math.max(
      0,
      opposingRightConflictProfile.yieldProgress - halfVehicleLengthPx,
    );
    vehicle.opposingRightTurnClearProgress = Math.min(
      vehicle.pathLength,
      opposingRightConflictProfile.clearProgress + halfVehicleLengthPx,
    );
  }
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
const leftTurnOpposingRightConflictProfiles = {};

function findPathConflictProgress(points, otherPoints, thresholdPx) {
  if (!points || !otherPoints || points.length < 2 || otherPoints.length < 2) {
    return null;
  }

  let cumulativeDistance = 0;
  const samplesPerSegment = 4;

  for (let i = 1; i < points.length; i += 1) {
    const start = points[i - 1];
    const end = points[i];
    const segmentLength = distanceBetween(start, end);
    const startStep = i === 1 ? 0 : 1;

    for (let step = startStep; step <= samplesPerSegment; step += 1) {
      const ratio = step / samplesPerSegment;
      const samplePoint = point(
        start.x + (end.x - start.x) * ratio,
        start.y + (end.y - start.y) * ratio,
      );
      const closest = findClosestPathPoint(otherPoints, samplePoint);

      if (closest && closest.distance <= thresholdPx) {
        return {
          progress: cumulativeDistance + segmentLength * ratio,
          point: samplePoint,
          otherProgress: closest.progress,
          otherPoint: closest.point,
        };
      }
    }

    cumulativeDistance += segmentLength;
  }

  return null;
}

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

function getLeftTurnOpposingRightConflictProfile(approachKey) {
  if (Object.prototype.hasOwnProperty.call(leftTurnOpposingRightConflictProfiles, approachKey)) {
    return leftTurnOpposingRightConflictProfiles[approachKey];
  }

  const opposingKey = getOpposingApproachKey(approachKey);
  const leftTurnPath = buildPath({ approach: approachKey, movement: "left" });
  const opposingRightTurnPath = buildPath({ approach: opposingKey, movement: "right" });
  const conflictLocation = findPathConflictProgress(
    leftTurnPath,
    opposingRightTurnPath,
    sharedLaneConflictThresholdPx,
  );

  if (!conflictLocation) {
    leftTurnOpposingRightConflictProfiles[approachKey] = null;
    return null;
  }

  leftTurnOpposingRightConflictProfiles[approachKey] = {
    yieldProgress: Math.max(0, conflictLocation.progress - leftTurnYieldBufferPx),
    clearProgress: Math.min(pathLength(leftTurnPath), conflictLocation.progress + leftTurnYieldBufferPx),
    opposingClearProgress: Math.min(
      pathLength(opposingRightTurnPath),
      conflictLocation.otherProgress + leftTurnYieldBufferPx,
    ),
  };

  return leftTurnOpposingRightConflictProfiles[approachKey];
}

function getVehicleAxisCoord(vehicle) {
  if (vehicle.state === "approach") {
    return vehicle.coord;
  }

  const approach = approaches[vehicle.approach];
  return approach.axis === "x" ? vehicle.position.x : vehicle.position.y;
}

function getSameApproachIntersectionLeader(vehicle) {
  const approach = approaches[vehicle.approach];
  let nearestLeader = null;

  for (const other of sim.vehicles) {
    if (other.id === vehicle.id || other.approach !== vehicle.approach || other.state !== "intersection") {
      continue;
    }

    if (other.progress > intersectionEntryAwarenessDistancePx) {
      continue;
    }

    const gapPx = getFollowingGapPx(
      approach,
      getVehicleAxisCoord(other),
      other.length,
      vehicle.coord,
      vehicle.length,
    );

    if (gapPx < -0.5) {
      continue;
    }

    if (!nearestLeader || gapPx < nearestLeader.gapPx) {
      nearestLeader = {
        vehicle: other,
        gapPx,
      };
    }
  }

  return nearestLeader;
}

function getThroughDistanceToConflict(vehicle, conflictCoord) {
  const approach = approaches[vehicle.approach];
  return (conflictCoord - getVehicleAxisCoord(vehicle)) * approach.travelSign;
}

function getNearestPedestrianConflict(vehicle) {
  if (vehicle.state !== "intersection" || !vehicle.path) {
    return null;
  }

  let nearestConflict = null;
  const halfVehicleLengthPx = (vehicle.length * pxPerMeter) / 2;

  for (const pedestrian of sim.pedestrians) {
    if (pedestrian.blockingVehicleId === vehicle.id) {
      continue;
    }

    const closest = findClosestPathPoint(vehicle.path, pedestrian.position);
    if (!closest || closest.distance > pedestrianPathConflictThresholdPx) {
      continue;
    }

    const distanceAheadPx = closest.progress - vehicle.progress;
    if (distanceAheadPx < -halfVehicleLengthPx) {
      continue;
    }

    const stopProgress = Math.max(vehicle.progress, closest.progress - halfVehicleLengthPx - pedestrianYieldBufferPx);
    if (!nearestConflict || stopProgress < nearestConflict.stopProgress) {
      nearestConflict = {
        pedestrian,
        stopProgress,
      };
    }
  }

  return nearestConflict;
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
      const intersectionLeader = leader ? null : getSameApproachIntersectionLeader(vehicle);
      let leadObject = null;

      if (leader) {
        const gap = getFollowingGapPx(approach, leader.coord, leader.length, vehicle.coord, vehicle.length);
        leadObject = {
          gap: Math.max(0.5, gap / pxPerMeter),
          speed: leader.v,
        };
      }

      if (intersectionLeader) {
        const entryLeader = {
          gap: Math.max(0.5, intersectionLeader.gapPx / pxPerMeter),
          speed: intersectionLeader.vehicle.v,
        };
        if (!leadObject || entryLeader.gap < leadObject.gap) {
          leadObject = entryLeader;
        }
      }

      if (!isVehiclePermitted(vehicle) || hasActiveCrosswalkPedestrian(approach.key)) {
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

      if (hasOpposingRightTurnConflict(vehicle)) {
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
      const intersectionLeader = leader ? null : getSameApproachIntersectionLeader(vehicle);
      vehicle.v = Math.max(0, vehicle.v + vehicle.a * dt);
      vehicle.coord += approach.travelSign * vehicle.v * pxPerMeter * dt;

      if (leader) {
        if (approach.travelSign > 0) {
          vehicle.coord = Math.min(vehicle.coord, leader.coord - minVehicleGapPx);
        } else {
          vehicle.coord = Math.max(vehicle.coord, leader.coord + minVehicleGapPx);
        }
      } else if (intersectionLeader) {
        const leaderCoord = getVehicleAxisCoord(intersectionLeader.vehicle);
        if (approach.travelSign > 0) {
          vehicle.coord = Math.min(vehicle.coord, leaderCoord - minVehicleGapPx);
        } else {
          vehicle.coord = Math.max(vehicle.coord, leaderCoord + minVehicleGapPx);
        }
        vehicle.v = Math.min(vehicle.v, intersectionLeader.vehicle.v);
      }

      vehicle.position = getVehiclePosition(approach, vehicle.coord);
      vehicle.angle = approach.pathDirection;
      updateVehicleViewCounts(vehicle);

      const canEnter =
        isVehiclePermitted(vehicle) &&
        !hasActiveCrosswalkPedestrian(approach.key) &&
        !hasOpposingThroughConflict(vehicle) &&
        !hasOpposingRightTurnConflict(vehicle) &&
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
    const opposingThroughYieldActive =
      vehicle.movement === "left" &&
      Number.isFinite(vehicle.yieldProgress) &&
      vehicle.progress <= vehicle.yieldProgress + 0.5 &&
      hasOpposingThroughConflict(vehicle);
    const opposingRightTurnYieldActive =
      vehicle.movement === "left" &&
      Number.isFinite(vehicle.opposingRightTurnYieldProgress) &&
      vehicle.progress <= vehicle.opposingRightTurnYieldProgress + 0.5 &&
      hasOpposingRightTurnConflict(vehicle);
    const pedestrianConflict = getNearestPedestrianConflict(vehicle);
    const shouldYieldInIntersection =
      opposingThroughYieldActive ||
      opposingRightTurnYieldActive ||
      Boolean(pedestrianConflict);
    let maxProgress = Number.POSITIVE_INFINITY;

    if (opposingThroughYieldActive) {
      maxProgress = Math.min(maxProgress, vehicle.yieldProgress);
    }
    if (opposingRightTurnYieldActive) {
      maxProgress = Math.min(maxProgress, vehicle.opposingRightTurnYieldProgress);
    }
    if (pedestrianConflict) {
      maxProgress = Math.min(maxProgress, pedestrianConflict.stopProgress);
    }

    if (shouldYieldInIntersection) {
      vehicle.v = Math.max(0, vehicle.v - 3.6 * dt);
    } else {
      vehicle.v = Math.min(vehicle.desiredSpeed, Math.max(vehicle.desiredSpeed * 0.65, vehicle.v + 1.4 * dt));
    }

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
    const { position } = sample;

    vehicle.position = position;
    vehicle.angle = sample.angle;
    updateVehicleViewCounts(vehicle);

    if (vehicle.progress < vehicle.pathLength) {
      remaining.push(vehicle);
      continue;
    }

    const freeFlowTravelTime = 150 / vehicle.desiredSpeed;
    const elapsedTravelTime = sim.time - vehicle.enteredAt;
    const delay = Math.max(0, elapsedTravelTime - freeFlowTravelTime);
    sim.stats.completedTrips += 1;
    sim.stats.totalDelay += delay;
  }

  sim.vehicles = remaining;
}

function updateVehicles(dt) {
  updateApproachVehicles(dt);
  updateIntersectionVehicles(dt);
  updatePedestrians(dt);

  const queueCount = getTotalRealTimeQueueCount();
  sim.stats.currentQueue = queueCount;
  sim.stats.queueTimeIntegral += queueCount * dt;
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
  if (sim.signal.state === "ew-all-red") {
    return "All Red (EW to NS)";
  }
  if (sim.signal.state === "ns-all-red") {
    return "All Red (NS to EW)";
  }
  return "North-South Yellow";
}

function updateOutputs() {
  sim.config = readConfig();
  controls.simSpeedLabel.textContent = `${sim.config.simSpeed.toFixed(1)}x`;
  statEls.arrivals.textContent = String(sim.stats.arrivals);
  statEls.departures.textContent = String(sim.stats.departures);
  statEls.queue.textContent = String(sim.stats.currentQueue);
  statEls.avgQueue.textContent = (sim.time > 0 ? sim.stats.queueTimeIntegral / sim.time : 0).toFixed(1);
  statEls.maxQueue.textContent = String(sim.stats.maxQueue);
  statEls.delay.textContent = `${(sim.stats.completedTrips ? sim.stats.totalDelay / sim.stats.completedTrips : 0).toFixed(1)} s`;
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
  ctx.lineTo(roadCenterX - centerlineSetbackPx, roadCenterY - 5);
  ctx.moveTo(roadCenterX + centerlineSetbackPx, roadCenterY - 5);
  ctx.lineTo(canvas.width, roadCenterY - 5);
  ctx.moveTo(0, roadCenterY + 5);
  ctx.lineTo(roadCenterX - centerlineSetbackPx, roadCenterY + 5);
  ctx.moveTo(roadCenterX + centerlineSetbackPx, roadCenterY + 5);
  ctx.lineTo(canvas.width, roadCenterY + 5);
  ctx.moveTo(roadCenterX - 5, 0);
  ctx.lineTo(roadCenterX - 5, roadCenterY - centerlineSetbackPx);
  ctx.moveTo(roadCenterX - 5, roadCenterY + centerlineSetbackPx);
  ctx.lineTo(roadCenterX - 5, canvas.height);
  ctx.moveTo(roadCenterX + 5, 0);
  ctx.lineTo(roadCenterX + 5, roadCenterY - centerlineSetbackPx);
  ctx.moveTo(roadCenterX + 5, roadCenterY + centerlineSetbackPx);
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
  ctx.fillRect(
    approaches.west.stopCoord - stopLineThicknessPx / 2,
    approaches.west.laneCenter - stopLineLengthPx / 2,
    stopLineThicknessPx,
    stopLineLengthPx
  );
  ctx.fillRect(
    approaches.east.stopCoord - stopLineThicknessPx / 2,
    approaches.east.laneCenter - stopLineLengthPx / 2,
    stopLineThicknessPx,
    stopLineLengthPx
  );
  ctx.fillRect(
    approaches.north.laneCenter - stopLineLengthPx / 2,
    approaches.north.stopCoord - stopLineThicknessPx / 2,
    stopLineLengthPx,
    stopLineThicknessPx
  );
  ctx.fillRect(
    approaches.south.laneCenter - stopLineLengthPx / 2,
    approaches.south.stopCoord - stopLineThicknessPx / 2,
    stopLineLengthPx,
    stopLineThicknessPx
  );

  drawCrosswalks();
}

function drawCrosswalks() {
  ctx.fillStyle = "rgba(255,255,255,0.78)";
  for (const crosswalk of Object.values(crosswalks)) {
    const isVertical = Math.abs(crosswalk.start.x - crosswalk.end.x) < 1e-6;
    const stripeCount = 6;
    for (let i = 0; i < stripeCount; i += 1) {
      const t = (i + 0.5) / stripeCount;
      const cx = crosswalk.start.x + (crosswalk.end.x - crosswalk.start.x) * t;
      const cy = crosswalk.start.y + (crosswalk.end.y - crosswalk.start.y) * t;
      if (isVertical) {
        ctx.fillRect(
          cx - crosswalkWidthPx / 2,
          cy - crosswalkStripeThicknessPx / 2,
          crosswalkWidthPx,
          crosswalkStripeThicknessPx
        );
      } else {
        ctx.fillRect(
          cx - crosswalkStripeThicknessPx / 2,
          cy - crosswalkWidthPx / 2,
          crosswalkStripeThicknessPx,
          crosswalkWidthPx
        );
      }
    }
  }
}

function drawPedestrian(pedestrian) {
  ctx.save();
  ctx.translate(pedestrian.position.x, pedestrian.position.y);

  const bodyRadius = pedestrian.bodyRadiusPx || pedBodyRadiusPx;
  const stridePhase = sim.time * (pedestrian.stepFrequencyHz || 1.7) * Math.PI * 2 + (pedestrian.stepPhase || 0);
  const gaitIntensity = clamp(
    (pedestrian.speedMps || 0) / Math.max(0.1, pedestrian.desiredSpeedMps || 1),
    0,
    1
  );
  const limbSwing = Math.sin(stridePhase) * bodyRadius * 0.42 * gaitIntensity;
  const bodyBob = (1 - Math.cos(stridePhase * 2)) * bodyRadius * 0.06 * gaitIntensity;
  ctx.translate(0, -bodyBob);

  ctx.strokeStyle = "#111827";
  ctx.lineWidth = 1.6;
  ctx.lineCap = "round";

  ctx.beginPath();
  ctx.moveTo(0, -bodyRadius * 0.15);
  ctx.lineTo(0, bodyRadius * 0.95);
  ctx.moveTo(0, bodyRadius * 0.25);
  ctx.lineTo(-limbSwing * 0.9, bodyRadius * 0.9);
  ctx.moveTo(0, bodyRadius * 0.25);
  ctx.lineTo(limbSwing * 0.9, bodyRadius * 0.9);
  ctx.moveTo(0, bodyRadius * 0.95);
  ctx.lineTo(-limbSwing * 0.75, bodyRadius * 1.95);
  ctx.moveTo(0, bodyRadius * 0.95);
  ctx.lineTo(limbSwing * 0.75, bodyRadius * 1.95);
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(0, 0, bodyRadius, 0, Math.PI * 2);
  ctx.fillStyle = "#0f172a";
  ctx.fill();

  ctx.beginPath();
  ctx.arc(0, -bodyRadius - 2.8, bodyRadius * 0.58, 0, Math.PI * 2);
  ctx.fillStyle = "#f4c7a1";
  ctx.fill();
  ctx.restore();
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
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  const horizontalLabelOffsetY = roadHalfWidth + 34;
  const verticalLabelOffsetX = roadHalfWidth + 34;

  ctx.fillText("Eastbound", roadCenterX - 150, roadCenterY + horizontalLabelOffsetY);

  ctx.fillText("Westbound", roadCenterX + 150, roadCenterY - horizontalLabelOffsetY);

  ctx.save();
  ctx.translate(roadCenterX - verticalLabelOffsetX, roadCenterY - 165);
  ctx.rotate(Math.PI / 2);
  ctx.fillText("Southbound", 0, 0);
  ctx.restore();

  ctx.save();
  ctx.translate(roadCenterX + verticalLabelOffsetX, roadCenterY + 165);
  ctx.rotate(Math.PI / 2);
  ctx.fillText("Northbound", 0, 0);
  ctx.restore();

  ctx.textAlign = "start";
  ctx.textBaseline = "alphabetic";
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
  sim.pedestrians.forEach(drawPedestrian);
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
    maybeSpawnPedestrians();
    updateVehicles(dt);
  }

  updateOutputs();
  drawScene();
  requestAnimationFrame(step);
}

function startSimulation() {
  sim.config = readConfig();
  sim.running = true;
  updateOutputs();
}

function pauseSimulation() {
  sim.running = false;
  updateOutputs();
}

function toggleSimulation() {
  if (sim.running) {
    pauseSimulation();
    return;
  }
  startSimulation();
}

controls.start.addEventListener("click", startSimulation);

controls.pause.addEventListener("click", pauseSimulation);

canvas.addEventListener("click", toggleSimulation);

controls.reset.addEventListener("click", resetSimulation);

[
  controls.green,
  controls.yellow,
  controls.allRed,
  controls.red,
  controls.arrivalRateWest,
  controls.arrivalRateEast,
  controls.arrivalRateNorth,
  controls.arrivalRateSouth,
  controls.pedVolumeTotal,
  controls.desiredSpeed,
  controls.leftTurnRatio,
  controls.rightTurnRatio,
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
    if (
      control === controls.pedVolumeTotal
    ) {
      approachList.forEach((approach) => scheduleNextPedArrival(approach.key));
    }
  });
});

resetSimulation();
requestAnimationFrame(step);

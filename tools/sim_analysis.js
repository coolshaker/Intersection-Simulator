const fs = require("fs");
const path = require("path");

const canvas = { width: 1000, height: 840 };
const pxPerMeter = 5.2;
const roadCenterX = 520;
const roadCenterY = canvas.height / 2;
const roadHalfWidth = 60;
const approachLength = 420;
const exitLength = 220;
const minVehicleGapPx = 12.0 * pxPerMeter + 8;
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
  const ratio = Math.min(1, Math.max(0, projection));
  const pointOnSegment = point(start.x + dx * ratio, start.y + dy * ratio);

  return {
    distance: distanceBetween(target, pointOnSegment),
    point: pointOnSegment,
    ratio,
  };
}

function getFollowingGapPx(approach, leaderCoord, leaderLength, followerCoord, followerLength) {
  const leaderRearCoord = leaderCoord - approach.travelSign * (leaderLength * pxPerMeter) / 2;
  const followerFrontCoord = followerCoord + approach.travelSign * (followerLength * pxPerMeter) / 2;
  return (leaderRearCoord - followerFrontCoord) * approach.travelSign;
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

function createRng(seed) {
  let state = seed >>> 0;
  return () => {
    state += 0x6D2B79F5;
    let t = state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
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

function sampleExponential(ratePerSecond, rng) {
  const u = Math.max(1e-9, 1 - rng());
  return -Math.log(u) / ratePerSecond;
}

function getVehiclePosition(approach, coord) {
  if (approach.axis === "x") {
    return point(coord, approach.laneCenter);
  }
  return point(approach.laneCenter, coord);
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

function createSimulation(config, seed) {
  const rng = createRng(seed);
  const leftTurnConflictProfiles = {};
  const leftTurnOpposingRightConflictProfiles = {};

  const sim = {
    rng,
    time: 0,
    vehicles: [],
    pedestrians: [],
    nextVehicleId: 1,
    nextPedestrianId: 1,
    nextArrivalTimes: {},
    nextPedArrivalTimes: {},
    pendingPedRequests: {},
    stats: {
      spawned: 0,
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
    config,
  };

  function scheduleNextArrival(approachKey) {
    const ratePerHour = sim.config.arrivalRatePerHour[approachKey];
    const ratePerSecond = ratePerHour / 3600;
    if (ratePerSecond <= 0) {
      sim.nextArrivalTimes[approachKey] = Number.POSITIVE_INFINITY;
      return;
    }
    sim.nextArrivalTimes[approachKey] = sim.time + sampleExponential(ratePerSecond, sim.rng);
  }

  function scheduleNextPedArrival(approachKey) {
    const ratePerHour = sim.config.pedestrianTotalRatePerHour / approachList.length;
    const ratePerSecond = ratePerHour / 3600;
    if (ratePerSecond <= 0) {
      sim.nextPedArrivalTimes[approachKey] = Number.POSITIVE_INFINITY;
      return;
    }
    sim.nextPedArrivalTimes[approachKey] = sim.time + sampleExponential(ratePerSecond, sim.rng);
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

  function pickMovement() {
    const leftTurnRatio = sim.config.leftTurnRatio;
    const rightTurnRatio = Math.min(sim.config.rightTurnRatio, Math.max(0, 1 - leftTurnRatio));
    const throughRatio = Math.max(0, 1 - leftTurnRatio - rightTurnRatio);
    const roll = sim.rng();
    if (roll < leftTurnRatio) {
      return "left";
    }
    if (roll < leftTurnRatio + throughRatio) {
      return "through";
    }
    return "right";
  }

  function createVehicle(approachKey) {
    const approach = approaches[approachKey];
    const movement = pickMovement();
    const desiredSpeed = sim.config.desiredSpeedMps * (0.88 + sim.rng() * 0.24);
    const isLargeVehicle = sim.rng() < sim.config.largeVehicleRatio;
    const vehicleLength = isLargeVehicle ? 12.0 : 5.2;
    const vehicleWidth = isLargeVehicle ? 2.5 : 2.0;

    return {
      id: sim.nextVehicleId++,
      approach: approachKey,
      movement,
      coord: approach.spawnCoord,
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
      position: getVehiclePosition(approach, approach.spawnCoord),
      angle: approach.pathDirection,
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
        sim.stats.spawned += 1;
        scheduleNextArrival(approach.key);
      }
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

  function isApproachVehicleGreen(approachKey) {
    if (sim.signal.state === "ew-green") {
      return approaches[approachKey].phaseGroup === "ew";
    }
    if (sim.signal.state === "ns-green") {
      return approaches[approachKey].phaseGroup === "ns";
    }
    return false;
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
    const baseOffsetPx = laneOffsetPx + (sim.rng() * 2 - 1) * pedestrianLateralOffsetPx * 0.35;
    const start = point(
      crosswalk.start.x + normal.x * baseOffsetPx,
      crosswalk.start.y + normal.y * baseOffsetPx,
    );
    const end = point(
      crosswalk.end.x + normal.x * baseOffsetPx,
      crosswalk.end.y + normal.y * baseOffsetPx,
    );
    const lengthPx = distanceBetween(start, end);
    const desiredSpeedMps = 0.95 + sim.rng() * 0.65;
    const bodyRadiusPx = pedBodyRadiusPx * (0.9 + sim.rng() * 0.22);
    const startDelayS = pedestrianStartDelayMinS +
      sim.rng() * (pedestrianStartDelayMaxS - pedestrianStartDelayMinS);

    return {
      id: sim.nextPedestrianId++,
      approach: approachKey,
      progress: 0,
      speedMps: 0,
      desiredSpeedMps,
      bodyRadiusPx,
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

      const canStartCrossing =
        !isApproachVehicleGreen(approach.key) &&
        isPedestrianEntryComfortable(approach.key);

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
      pedestrian.start.y + (pedestrian.end.y - pedestrian.start.y) * ratio + pedestrian.normal.y * dodgeOffsetPx,
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
        currentProgress + pedestrian.speedMps * pxPerMeter * dt,
      );

      let chosenProgress = desiredProgress;
      let chosenDodgeTarget = 0;
      let blocker = getBlockingVehicleForPedestrian(
        pedestrian,
        getPedestrianPositionAtProgress(pedestrian, desiredProgress, 0),
      );

      if (blocker) {
        const dodgeCandidates = getPedestrianDodgeCandidates(pedestrian, blocker);
        const forwardDodge = dodgeCandidates.find((offset) => (
          !getBlockingVehicleForPedestrian(
            pedestrian,
            getPedestrianPositionAtProgress(pedestrian, desiredProgress, offset),
          )
        ));

        if (typeof forwardDodge === "number") {
          chosenDodgeTarget = forwardDodge;
        } else {
          chosenProgress = currentProgress;
          const lateralDodge = dodgeCandidates.find((offset) => (
            !getBlockingVehicleForPedestrian(
              pedestrian,
              getPedestrianPositionAtProgress(pedestrian, currentProgress, offset),
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
          pedestrianDodgeRatePxPerSecond * dt,
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

  function getVehicleCanvasHalfExtents(vehicle) {
    const halfLengthPx = (vehicle.length * pxPerMeter) / 2;
    const halfWidthPx = (vehicle.width * pxPerMeter) / 2;

    return {
      x: Math.abs(Math.cos(vehicle.angle)) * halfLengthPx + Math.abs(Math.sin(vehicle.angle)) * halfWidthPx,
      y: Math.abs(Math.sin(vehicle.angle)) * halfLengthPx + Math.abs(Math.cos(vehicle.angle)) * halfWidthPx,
    };
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
      vehicle.position = sample.position;
      vehicle.angle = sample.angle;

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

  function step(dt) {
    sim.time += dt;
    updateSignal(dt);
    maybeSpawnVehicles();
    maybeSpawnPedestrians();
    updateVehicles(dt);
  }

  approachList.forEach((approach) => {
    scheduleNextArrival(approach.key);
    scheduleNextPedArrival(approach.key);
    sim.pendingPedRequests[approach.key] = 0;
  });

  return {
    sim,
    step,
  };
}

function runScenarioReplication(scenario, replicationIndex) {
  const seed = scenario.seedBase + replicationIndex * 101;
  const { sim, step } = createSimulation(scenario.config, seed);
  const dt = scenario.dtSeconds;
  const totalSteps = Math.ceil(scenario.durationSeconds / dt);

  for (let i = 0; i < totalSteps; i += 1) {
    step(dt);
  }

  const simTime = sim.time || scenario.durationSeconds;
  return {
    seed,
    avgDelaySeconds: sim.stats.completedTrips ? sim.stats.totalDelay / sim.stats.completedTrips : 0,
    avgQueueVehicles: simTime > 0 ? sim.stats.queueTimeIntegral / simTime : 0,
    maxQueueVehicles: sim.stats.maxQueue,
    completedTrips: sim.stats.completedTrips,
    spawnedVehicles: sim.stats.spawned,
    throughputVehPerHour: simTime > 0 ? sim.stats.completedTrips * 3600 / simTime : 0,
  };
}

function mean(values) {
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

function round(value, digits = 1) {
  const factor = 10 ** digits;
  return Math.round(value * factor) / factor;
}

function summarizeScenario(scenario) {
  const replications = Array.from({ length: scenario.replications }, (_, index) =>
    runScenarioReplication(scenario, index),
  );

  return {
    id: scenario.id,
    title: scenario.title,
    description: scenario.description,
    timing: {
      eastWestGreen: scenario.config.eastWestGreen,
      yellow: scenario.config.yellow,
      allRed: scenario.config.allRed,
      northSouthGreen: scenario.config.northSouthGreen,
    },
    demandVehPerHour: scenario.config.arrivalRatePerHour,
    pedestrianTotalRatePerHour: scenario.config.pedestrianTotalRatePerHour,
    replications: scenario.replications,
    durationSeconds: scenario.durationSeconds,
    meanAvgDelaySeconds: round(mean(replications.map((item) => item.avgDelaySeconds))),
    meanAvgQueueVehicles: round(mean(replications.map((item) => item.avgQueueVehicles))),
    meanMaxQueueVehicles: round(mean(replications.map((item) => item.maxQueueVehicles))),
    meanCompletedTrips: round(mean(replications.map((item) => item.completedTrips))),
    meanThroughputVehPerHour: round(mean(replications.map((item) => item.throughputVehPerHour))),
    rawReplications: replications,
  };
}

const scenarios = [
  {
    id: "S1",
    title: "Baseline Default Settings",
    description: "Default browser settings with balanced approach demand and baseline pedestrian activity.",
    replications: 10,
    durationSeconds: 1200,
    dtSeconds: 0.1,
    seedBase: 1000,
    config: {
      eastWestGreen: 30,
      yellow: 3,
      allRed: 2,
      northSouthGreen: 30,
      arrivalRatePerHour: {
        west: 300,
        east: 300,
        north: 300,
        south: 300,
      },
      pedestrianTotalRatePerHour: 100,
      desiredSpeedMps: 30 * 0.44704,
      leftTurnRatio: 0.05,
      rightTurnRatio: 0.05,
      largeVehicleRatio: 0.05,
    },
  },
  {
    id: "S2",
    title: "Higher Left-Turn Activity",
    description: "Balanced demand with a higher left-turn share to emphasize permissive turning effects.",
    replications: 10,
    durationSeconds: 1200,
    dtSeconds: 0.1,
    seedBase: 2000,
    config: {
      eastWestGreen: 30,
      yellow: 3,
      allRed: 2,
      northSouthGreen: 30,
      arrivalRatePerHour: {
        west: 300,
        east: 300,
        north: 300,
        south: 300,
      },
      pedestrianTotalRatePerHour: 100,
      desiredSpeedMps: 30 * 0.44704,
      leftTurnRatio: 0.30,
      rightTurnRatio: 0.05,
      largeVehicleRatio: 0.05,
    },
  },
  {
    id: "S3",
    title: "Higher Truck Proportion",
    description: "Balanced demand with a larger share of heavy vehicles.",
    replications: 10,
    durationSeconds: 1200,
    dtSeconds: 0.1,
    seedBase: 3000,
    config: {
      eastWestGreen: 30,
      yellow: 3,
      allRed: 2,
      northSouthGreen: 30,
      arrivalRatePerHour: {
        west: 300,
        east: 300,
        north: 300,
        south: 300,
      },
      pedestrianTotalRatePerHour: 100,
      desiredSpeedMps: 30 * 0.44704,
      leftTurnRatio: 0.05,
      rightTurnRatio: 0.05,
      largeVehicleRatio: 0.30,
    },
  },
  {
    id: "S4",
    title: "Higher Pedestrian Demand",
    description: "Balanced vehicle demand with a higher crosswalk arrival rate.",
    replications: 10,
    durationSeconds: 1200,
    dtSeconds: 0.1,
    seedBase: 4000,
    config: {
      eastWestGreen: 30,
      yellow: 3,
      allRed: 2,
      northSouthGreen: 30,
      arrivalRatePerHour: {
        west: 300,
        east: 300,
        north: 300,
        south: 300,
      },
      pedestrianTotalRatePerHour: 500,
      desiredSpeedMps: 30 * 0.44704,
      leftTurnRatio: 0.05,
      rightTurnRatio: 0.05,
      largeVehicleRatio: 0.05,
    },
  },
  {
    id: "S5",
    title: "Heavier East-West Demand With Base Timing",
    description: "Directional demand imbalance is introduced while keeping the baseline timing plan.",
    replications: 10,
    durationSeconds: 1200,
    dtSeconds: 0.1,
    seedBase: 5000,
    config: {
      eastWestGreen: 30,
      yellow: 3,
      allRed: 2,
      northSouthGreen: 30,
      arrivalRatePerHour: {
        west: 600,
        east: 600,
        north: 300,
        south: 300,
      },
      pedestrianTotalRatePerHour: 100,
      desiredSpeedMps: 30 * 0.44704,
      leftTurnRatio: 0.05,
      rightTurnRatio: 0.05,
      largeVehicleRatio: 0.05,
    },
  },
  {
    id: "S6",
    title: "Heavier East-West Demand With Retiming",
    description: "The east-west-heavy demand case is retimed by reallocating green time toward the dominant phase.",
    replications: 10,
    durationSeconds: 1200,
    dtSeconds: 0.1,
    seedBase: 6000,
    config: {
      eastWestGreen: 40,
      yellow: 3,
      allRed: 2,
      northSouthGreen: 20,
      arrivalRatePerHour: {
        west: 600,
        east: 600,
        north: 300,
        south: 300,
      },
      pedestrianTotalRatePerHour: 100,
      desiredSpeedMps: 30 * 0.44704,
      leftTurnRatio: 0.05,
      rightTurnRatio: 0.05,
      largeVehicleRatio: 0.05,
    },
  },
];

const summary = {
  project: "Intersection Simulator",
  generatedAt: new Date().toISOString(),
  assumptions: {
    laneConfiguration: "One inbound lane per approach at a four-leg signalized intersection.",
    control: "Two-phase signal control with user-adjustable green, yellow, and all-red intervals.",
    behavior: "Stochastic arrivals, mixed turning movements, pedestrian crossings, large-vehicle share, and permissive left-turn yielding.",
    resultsNote: "Scenario results are illustrative outputs from the teaching simulator rather than calibrated field estimates.",
  },
  scenarios: scenarios.map(summarizeScenario),
};

const outputPath = process.argv[2];
const payload = JSON.stringify(summary, null, 2);

if (outputPath) {
  fs.mkdirSync(path.dirname(outputPath), { recursive: true });
  fs.writeFileSync(outputPath, payload);
} else {
  process.stdout.write(payload);
}

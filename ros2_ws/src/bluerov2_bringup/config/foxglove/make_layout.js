#!/usr/bin/env node
/**
 * Generates the Foxglove layout for the BlueROV2 sensor stack.
 *
 * Foxglove imports layouts as JSON only, so this script builds the layout and writes
 * `rov_layout.json` next to it. Edit the panels here, then regenerate:
 *
 *   node make_layout.js [namespace]      # default namespace: /bluerov2
 *
 * In Foxglove: Layout -> Import from file -> rov_layout.json.
 *
 * Tabs: Overview | IMU | DVL | Cameras | Stereo | 3D + log.
 */

const fs = require("fs");
const path = require("path");

const NS = (process.argv[2] || "/bluerov2").replace(/\/$/, "");
const t = (topic) => `${NS}/${topic}`;

// Series colours, one per axis, reused across all plots so x/y/z always look the same.
const AXIS = { x: "#e5484d", y: "#30a46c", z: "#3e63dd" };
const EXTRA = ["#f76b15", "#8e4ec6", "#12a594", "#d6409f"];

const panels = {};
let seq = 0;

/** Registers a panel config and returns its id, for use in the layout tree. */
function add(type, config) {
  const id = `${type}!rov${seq++}`;
  panels[id] = config;
  return id;
}

function series(value, label, color) {
  return { value, label, color, enabled: true, timestampMethod: "receiveTime" };
}

/** Time-series plot; the x axis follows the last `window` seconds. */
function plot(title, paths, window = 30) {
  return add("Plot", {
    title,
    paths,
    showLegend: true,
    legendDisplay: "floating",
    showXAxisLabels: true,
    showYAxisLabels: true,
    isSynced: true,
    xAxisVal: "timestamp",
    followingViewWidth: window,
  });
}

/** Plot of x, y, z of a vector field, e.g. `imu/data_raw.angular_velocity`. */
function xyzPlot(title, base, labels = ["x", "y", "z"]) {
  return plot(
    title,
    ["x", "y", "z"].map((a, i) => series(`${base}.${a}`, labels[i], AXIS[a]))
  );
}

function image(topic, calibration) {
  return add("Image", {
    imageMode: {
      imageTopic: topic,
      calibrationTopic: calibration,
      synchronize: false,
      rotation: 0,
      flipHorizontal: false,
      flipVertical: false,
    },
    transformMarkers: false,
  });
}

/** Compressed image topic of a camera base, e.g. `camera` or `multicam/aux_left`. */
const cam = (base) => image(t(`${base}/image_raw/compressed`), t(`${base}/camera_info`));

function indicator(pathValue, onLabel, offLabel) {
  return add("Indicator", {
    path: pathValue,
    style: "background",
    fallbackColor: "#e5484d",
    fallbackLabel: offLabel,
    rules: [{ operator: "=", rawValue: "true", color: "#30a46c", label: onLabel }],
  });
}

function raw(topicPath) {
  return add("RawMessages", { topicPath, diffEnabled: false, expansion: "none" });
}

/** Binary split. `dir` is "row" (side by side) or "column" (stacked). */
function split(dir, first, second, pct = 50) {
  return { first, second, direction: dir, splitPercentage: pct };
}

/** Splits `items` evenly along `dir`. */
function even(dir, items) {
  if (items.length === 1) return items[0];
  return split(dir, items[0], even(dir, items.slice(1)), 100 / items.length);
}

// ---------------------------------------------------------------- IMU
const imuGyro = () =>
  xyzPlot("IMU angular velocity [rad/s]", t("imu/data_raw.angular_velocity"),
    ["roll rate", "pitch rate", "yaw rate"]);
const imuAccel = () =>
  xyzPlot("IMU specific force [m/s²]", t("imu/data_raw.linear_acceleration"),
    ["ax", "ay", "az (≈ +9.81 at rest)"]);
const imuAttitude = () =>
  plot("IMU attitude [deg] (from imu/data orientation)", [
    series(`${t("imu/data.orientation")}.@rpy.roll.@degrees`, "roll", AXIS.x),
    series(`${t("imu/data.orientation")}.@rpy.pitch.@degrees`, "pitch", AXIS.y),
    series(`${t("imu/data.orientation")}.@rpy.yaw.@degrees`, "yaw", AXIS.z),
  ]);
const imuMag = () =>
  xyzPlot("Magnetic field [T]", t("imu/mag.magnetic_field"));
const imuPressure = () =>
  plot("Hull pressure [Pa] (not depth)", [
    series(t("imu/pressure.fluid_pressure"), "pressure", EXTRA[0]),
  ], 120);
const imuTemp = () =>
  plot("IMU temperature [°C]", [
    series(t("imu/temperature.temperature"), "temperature", EXTRA[1]),
  ], 120);

// ---------------------------------------------------------------- DVL
const dvlVel = () =>
  xyzPlot("DVL bottom-track velocity [m/s]", t("dvl/velocity.twist.twist.linear"),
    ["surge", "sway", "heave"]);
const dvlLock = () => indicator(t("dvl/report.velocity_valid"), "DVL BOTTOM LOCK", "NO BOTTOM LOCK");
const dvlAltitude = () =>
  plot("DVL altitude [m]", [series(t("dvl/report.altitude"), "altitude", EXTRA[2])], 60);
const dvlFom = () =>
  plot("DVL figure of merit [m/s] (lower is better)", [
    series(t("dvl/report.fom"), "FOM", EXTRA[0]),
  ], 60);
const dvlBeams = (field, title) =>
  plot(title, [0, 1, 2, 3].map((i) =>
    series(`${t("dvl/report")}.beams[${i}].${field}`, `beam ${i}`, EXTRA[i])), 60);
const dvlDeadReckoning = () =>
  xyzPlot("DVL dead reckoning position [m] (drifts; reference only)",
    t("dvl/dead_reckoning"), ["x", "y", "z"]);
const dvlTrack = () =>
  add("Plot", {
    title: "DVL dead reckoning track, x vs y [m]",
    paths: [series(t("dvl/dead_reckoning.y"), "track", EXTRA[1])],
    showLegend: false,
    showXAxisLabels: true,
    showYAxisLabels: true,
    isSynced: false,
    xAxisVal: "custom",
    xAxisPath: { value: t("dvl/dead_reckoning.x"), enabled: true },
  });

// ---------------------------------------------------------------- Cameras
const MULTICAMS = ["aux_right", "aux_left", "bottom_most", "stereo_bottom"];

// ---------------------------------------------------------------- 3D + log
const scene = () =>
  add("3D", {
    cameraState: {
      distance: 3, perspective: true, phi: 60, thetaOffset: 45,
      targetOffset: [0, 0, 0], target: [0, 0, 0], fovy: 45, near: 0.01, far: 5000,
    },
    followMode: "follow-pose",
    followTf: "base_link",
    scene: { enableStats: false, transforms: { showLabel: true, axisScale: 0.3 } },
    topics: { [t("dvl/altitude")]: { visible: true } },
    layers: {
      grid: { layerId: "foxglove.Grid", size: 10, divisions: 20, frameId: "base_link" },
    },
    publish: { type: "point" },
  });
const log = () => add("RosOut", { searchTerms: [], minLogLevel: 2 });

// ---------------------------------------------------------------- Tabs
const tabs = [
  {
    title: "Overview",
    layout: split("row",
      split("column",
        cam("camera"),
        split("row", cam("multicam/aux_right"), cam("multicam/aux_left")),
        60),
      even("column", [
        split("column", dvlLock(), dvlVel(), 12),
        imuGyro(),
        imuAccel(),
        dvlAltitude(),
      ]),
      58),
  },
  {
    title: "IMU",
    layout: split("row",
      even("column", [imuGyro(), imuAccel(), imuAttitude()]),
      even("column", [imuMag(), imuPressure(), imuTemp()]),
      55),
  },
  {
    title: "DVL",
    layout: split("row",
      even("column", [
        split("column", dvlLock(), dvlVel(), 15),
        split("row", dvlAltitude(), dvlFom()),
        dvlBeams("distance", "Beam range [m]"),
        dvlBeams("rssi", "Beam RSSI [dBm]"),
      ]),
      split("column",
        split("row", dvlTrack(), dvlDeadReckoning()),
        raw(t("dvl/report")),
        50),
      58),
  },
  {
    title: "Cameras",
    layout: split("column",
      cam("camera"),
      split("column",
        split("row", cam(`multicam/${MULTICAMS[0]}`), cam(`multicam/${MULTICAMS[1]}`)),
        split("row", cam(`multicam/${MULTICAMS[2]}`), cam(`multicam/${MULTICAMS[3]}`))),
      40),
  },
  {
    // Both stereo pairs, large: top pair above, bottom pair below.
    title: "Stereo",
    layout: split("column",
      split("row", cam(`multicam/${MULTICAMS[0]}`), cam(`multicam/${MULTICAMS[1]}`)),
      split("row", cam(`multicam/${MULTICAMS[2]}`), cam(`multicam/${MULTICAMS[3]}`))),
  },
  {
    title: "3D + log",
    layout: split("row", scene(), log(), 60),
  },
];

const tabId = add("Tab", { activeTabIdx: 0, tabs });

const layout = {
  configById: panels,
  globalVariables: {},
  userNodes: {},
  playbackConfig: { speed: 1 },
  layout: tabId,
};

const out = path.join(__dirname, "rov_layout.json");
fs.writeFileSync(out, JSON.stringify(layout, null, 2) + "\n");
console.log(`wrote ${out}: ${Object.keys(panels).length} panels, namespace ${NS}`);

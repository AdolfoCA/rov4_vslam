#!/usr/bin/env node
/**
 * Generates the Foxglove layout for the BlueROV2 sensor stack.
 *
 * Foxglove imports layouts as JSON only, so this script builds the layout and writes
 * `rov_layout.json` next to it. Edit the panels here, then regenerate:
 *
 *   cd ros2_ws/src/foxglove && node make_layout.js [namespace]   # default: /bluerov2
 *
 * In Foxglove: Layout -> Import from file -> rov_layout.json.
 *
 * Tabs: Overview | Record | DVL | Cameras | 3D. Kept to the essentials on purpose.
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

// ---------------------------------------------------------------- Health
// dvl_health publishes its checks as diagnostics on dvl/health: status[0] is the overall
// verdict (level 0 GOOD, 1 CHECK, 2 BAD), then one line per check.
const HEALTH = t("dvl/health");
const dvlOverall = () =>
  add("Indicator", {
    path: `${HEALTH}.status[0].level`,
    style: "background",
    fallbackColor: "#e5484d",
    fallbackLabel: "DVL: BAD (or no health data)",
    rules: [
      { operator: "=", rawValue: "0", color: "#30a46c", label: "DVL: GOOD" },
      { operator: "=", rawValue: "1", color: "#f5a524", label: "DVL: CHECK" },
    ],
  });
const dvlChecks = () =>
  add("DiagnosticSummary", {
    topicToRender: HEALTH,
    minLevel: 0,
    pinnedIds: [],
    hardwareIdFilter: "",
    sortByLevel: false,
  });
const dvlCheckDetail = () =>
  add("DiagnosticStatusPanel", {
    topicToRender: HEALTH,
    selectedHardwareId: "dvl_a50",
    selectedName: "DVL: overall",
    collapsedSections: [],
  });

// ---------------------------------------------------------------- Plots
const dvlVel = () =>
  xyzPlot("DVL velocity [m/s]", t("dvl/velocity.twist.twist.linear"),
    ["forward", "left", "up"]);
const dvlAltitude = () =>
  plot("DVL altitude [m]", [series(t("dvl/report.altitude"), "altitude", EXTRA[2])], 60);
const dvlBeamRanges = () =>
  plot("DVL beam ranges [m] (similar on a flat floor)", [0, 1, 2, 3].map((i) =>
    series(`${t("dvl/report")}.beams[${i}].distance`, `beam ${i}`, EXTRA[i])), 60);
const imuAttitude = () =>
  plot("Attitude [deg]", [
    series(`${t("imu/data.orientation")}.@rpy.roll.@degrees`, "roll", AXIS.x),
    series(`${t("imu/data.orientation")}.@rpy.pitch.@degrees`, "pitch", AXIS.y),
    series(`${t("imu/data.orientation")}.@rpy.yaw.@degrees`, "yaw", AXIS.z),
  ], 60);

// ---------------------------------------------------------------- Recording
// recording_manager (started by bluerov2.launch.py) runs record.launch.py: stream check,
// then one MCAP bag in <data>/<folder>/rov_YYYYmmdd_HHMMSS.
const REC = t("recording");
const recActive = () =>
  add("Indicator", {
    path: `${REC}/active.data`,
    style: "background",
    fallbackColor: "#6f6f6f",
    fallbackLabel: "not recording",
    rules: [{ operator: "=", rawValue: "true", color: "#e5484d", label: "● RECORDING" }],
  });
const recButton = (verb, color) =>
  add("CallService", {
    serviceName: `${REC}/${verb}`,
    requestPayload: "{}",
    layout: "vertical",
    buttonText: verb === "start" ? "Start recording" : "Stop recording",
    buttonTooltip: verb === "start"
      ? "Checks the streams, then records into the folder below"
      : "Stops and closes the bag cleanly",
    buttonColor: color,
    editingMode: false,
  });
const recFolder = () =>
  add("Publish", {
    topicName: `${REC}/set_folder`,
    datatype: "std_msgs/msg/String",
    buttonText: "Set folder",
    buttonTooltip: "Subfolder of the data directory (the project's data/ folder)",
    buttonColor: "#3e63dd",
    advancedView: true,
    value: JSON.stringify({ data: "pool_test" }, null, 2),
  });
const recStatus = () => raw(`${REC}/status.data`);
// Compact bar for the Overview: state, start, stop.
const recBar = () =>
  split("row", recActive(), split("row", recButton("start", "#30a46c"),
    recButton("stop", "#e5484d")), 40);

// ---------------------------------------------------------------- Cameras
const MULTICAMS = ["aux_right", "aux_left", "bottom_most", "stereo_bottom"];

// ---------------------------------------------------------------- 3D + log
// Display frame odom (published by dead_reckoning_node): the tracks stay put and the
// vehicle (base_link and its sensor frames) moves over them.
//   orange  dead_reckoning/path      IMU heading + DVL velocity
//   purple  dead_reckoning/dvl_path  the DVL's own dead reckoning (its own gyro)
// Where the two diverge, one of the two headings is wrong.
const DR_COLOR = "#f76b15";
const DVL_DR_COLOR = "#8e4ec6";
const scene = () =>
  add("3D", {
    cameraState: {
      distance: 20, perspective: true, phi: 30, thetaOffset: 45,
      targetOffset: [0, 0, 0], target: [0, 0, 0], fovy: 45, near: 0.01, far: 5000,
    },
    followMode: "follow-none",
    followTf: "odom",
    scene: { enableStats: false, transforms: { showLabel: true, axisScale: 0.3 } },
    topics: {
      [t("dvl/altitude")]: { visible: true },
      [t("dead_reckoning/path")]: {
        visible: true, type: "line", lineWidth: 0.05, gradient: [DR_COLOR, DR_COLOR],
      },
      [t("dead_reckoning/dvl_path")]: {
        visible: true, type: "line", lineWidth: 0.05, gradient: [DVL_DR_COLOR, DVL_DR_COLOR],
      },
      [t("dead_reckoning/odometry")]: {
        visible: true, type: "arrow", color: DR_COLOR, arrowScale: [1, 0.15, 0.15],
      },
    },
    layers: {
      grid: { layerId: "foxglove.Grid", size: 50, divisions: 50, frameId: "odom" },
    },
    publish: { type: "point" },
  });
const log = () => add("RosOut", { searchTerms: [], minLogLevel: 2 });

// ---------------------------------------------------------------- Tabs
const tabs = [
  {
    // Everything needed during a dive, on one screen.
    title: "Overview",
    layout: split("row",
      split("column",
        cam("camera"),
        split("row", cam("multicam/aux_left"), cam("multicam/aux_right")),
        55),
      split("column",
        recBar(),
        even("column", [
          split("column", dvlOverall(), dvlChecks(), 15),
          dvlVel(),
          dvlAltitude(),
          imuAttitude(),
        ]),
        12),
      60),
  },
  {
    // Where to record, start/stop, and what the recorder is doing.
    title: "Record",
    layout: split("column",
      recBar(),
      split("row", recFolder(), recStatus(), 45),
      25),
  },
  {
    // Is the DVL data good? Checklist on the left, the raw signals on the right.
    title: "DVL",
    layout: split("row",
      split("column",
        split("column", dvlOverall(), dvlChecks(), 12),
        dvlCheckDetail(),
        55),
      even("column", [dvlVel(), dvlAltitude(), dvlBeamRanges()]),
      45),
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
    title: "3D",
    layout: split("row", scene(), log(), 70),
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

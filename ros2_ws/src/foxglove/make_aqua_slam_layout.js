#!/usr/bin/env node
/**
 * Generates the Foxglove layout for AQUA-SLAM (ros2_ws/src/aqua_slam), the Foxglove
 * counterpart of the original rviz view (AQUA-SLAM/launch/falcon_tightly.rviz).
 *
 *   cd ros2_ws/src/foxglove && node make_aqua_slam_layout.js [node]   # default: /aqua_slam
 *
 * In Foxglove: Layout -> Import from file -> aqua_slam_layout.json.
 *
 * One screen:
 *   top     left image with the ORB features (img_with_info) | right image with the
 *           stereo matches of the same points (img_with_info_right). The images are
 *           rectified, so a pair lies on the same row in the two images.
 *             green = point matched to the map, blue = visual-odometry point,
 *             yellow (right, before initialisation) = any stereo pair found
 *   bottom  3D map in the aqua_slam_map frame (gravity-aligned, origin at the start):
 *             sparse_map        map points seen by >= 5 keyframes, coloured per map
 *             markers           keyframes (green, magenta = poor vision)
 *             orb_path          keyframe trajectory after optimisation (blue)
 *             integration_path  DVL + gyro dead reckoning (red)
 *             orb_pose          current camera pose (arrow) + TF aqua_slam_camera
 *
 * Map, paths and markers appear only after the DVL-gyro initialisation (the vehicle has
 * to move IMUInitTranslation metres); the images appear from the first frame.
 */

const fs = require("fs");
const path = require("path");

const NODE = (process.argv[2] || "/aqua_slam").replace(/\/$/, "");
const t = (topic) => `${NODE}/${topic}`;
const MAP_FRAME = "aqua_slam_map";

const SLAM_COLOR = "#3e63dd"; // blue
const DVL_COLOR = "#e5484d"; // red

const panels = {};
let seq = 0;
function add(type, config) {
  const id = `${type}!aqua${seq++}`;
  panels[id] = config;
  return id;
}
const split = (direction, first, second, splitPercentage = 50) => ({
  first, second, direction, splitPercentage,
});

/** Image panel on the compressed topic (image_transport), much lighter on the websocket. */
const image = (topic) =>
  add("Image", {
    imageMode: {
      imageTopic: `${t(topic)}/compressed`,
      synchronize: false, rotation: 0, flipHorizontal: false, flipVertical: false,
    },
    transformMarkers: false,
  });

const scene = add("3D", {
  cameraState: {
    distance: 6, perspective: true, phi: 50, thetaOffset: 45,
    targetOffset: [0, 0, 0], target: [0, 0, 0], fovy: 45, near: 0.01, far: 500,
  },
  followMode: "follow-none",
  followTf: MAP_FRAME,
  scene: { enableStats: false, transforms: { showLabel: true, axisScale: 0.2 } },
  topics: {
    [t("sparse_map")]: {
      visible: true, pointSize: 3, pointShape: "circle",
      colorMode: "rgb", colorField: "rgb", decayTime: 0,
    },
    [t("markers")]: { visible: true },
    [t("orb_path")]: {
      visible: true, type: "line", lineWidth: 0.02, gradient: [SLAM_COLOR, SLAM_COLOR],
    },
    [t("integration_path")]: {
      visible: true, type: "line", lineWidth: 0.02, gradient: [DVL_COLOR, DVL_COLOR],
    },
    [t("orb_pose")]: {
      visible: true, type: "arrow", color: SLAM_COLOR, arrowScale: [0.3, 0.05, 0.05],
    },
  },
  layers: {
    grid: { layerId: "foxglove.Grid", size: 10, divisions: 20, frameId: MAP_FRAME },
  },
  publish: { type: "point" },
});

const layout = {
  configById: panels,
  globalVariables: {},
  userNodes: {},
  playbackConfig: { speed: 1 },
  layout: split("column",
    split("row", image("img_with_info"), image("img_with_info_right")),
    scene,
    40),
};

const out = path.join(__dirname, "aqua_slam_layout.json");
fs.writeFileSync(out, JSON.stringify(layout, null, 2) + "\n");
console.log(`wrote ${out}: ${Object.keys(panels).length} panels, node ${NODE}`);

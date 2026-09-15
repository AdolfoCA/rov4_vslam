#!/usr/bin/env bash
# Create the GitHub repository for this project and push the first commit.
#
#   cd rov4_vslam
#   ./setup_github.sh
#
# Safe to re-run: it skips whatever is already done. It will not force-push and it
# will not touch an existing remote called 'origin' without telling you.

set -euo pipefail

REPO_NAME="rov4_vslam"
VISIBILITY="public"          # change to 'private' if you would rather start closed
DESCRIPTION="ROS 2 Humble driver stack for a BlueROV2 Heavy: Navigator IMU over MAVLink, Water Linked DVL-A50, and the monocular camera, in Docker."

cd "$(dirname "$0")"

# --- git identity -------------------------------------------------------------------
# Only set locally, so this never changes your global git config.
git rev-parse --git-dir >/dev/null 2>&1 || { echo "Initialising repository"; git init -q -b main; }
git config user.name  >/dev/null 2>&1 || git config user.name  "Adolfo Damiano Cafaro"
git config user.email >/dev/null 2>&1 || git config user.email "adaca@dtu.dk"

# --- first commit -----------------------------------------------------------------
if git rev-parse HEAD >/dev/null 2>&1; then
  echo "Repository already has commits; leaving history alone."
else
  git add -A
  git commit -q -F - <<'MSG'
BlueROV2 Heavy ROS 2 Humble sensor stack

Dockerised ROS 2 Humble workspace publishing the three sensors of a BlueROV2
Heavy: the Navigator IMU over MAVLink, a Water Linked DVL-A50 over its TCP JSON
protocol, and the monocular low-light camera from the BlueOS H.264 UDP stream.

Packages:
  bluerov2_msgs     DVL report, per-beam and dead-reckoning message definitions
  bluerov2_imu      pymavlink driver, NED/FRD to ENU/FLU conversion, clock offset
                    estimation by sliding-window minimum filter
  bluerov2_dvl      TCP JSON client with reconnect, per-beam data and FOM
  bluerov2_camera   GStreamer appsink pipeline, Image + CameraInfo
  bluerov2_tf       static extrinsics from one validated config file
  bluerov2_bringup  launch, parameters, Foxglove bridge

Each driver keeps its pure logic in a module with no ROS dependencies so it can
be tested without a ROS graph or hardware: 60 tests cover the frame rotations,
the clock estimator, the DVL wire format, the MAVLink unit scalings and the TF
tree validation.

Network addresses, sensor extrinsics, camera calibration and IMU noise figures
are placeholders marked TODO; see README section 7.
MSG
  echo "Created the initial commit ($(git rev-list --count HEAD) commit, $(git ls-files | wc -l | tr -d ' ') files)"
fi

# --- remote -------------------------------------------------------------------------
if git remote get-url origin >/dev/null 2>&1; then
  echo "Remote 'origin' already set to $(git remote get-url origin) — pushing to it."
  git push -u origin main
  exit 0
fi

if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
  gh repo create "$REPO_NAME" \
    --"$VISIBILITY" \
    --source=. \
    --remote=origin \
    --push \
    --description "$DESCRIPTION"
  echo
  echo "Done: $(gh repo view --json url -q .url)"
else
  OWNER="$(git config user.name >/dev/null 2>&1 && echo AdolfoCA || echo AdolfoCA)"
  cat <<EOF

The GitHub CLI is not installed or not signed in, so the repository has to be
created by hand. Two steps:

  1. Open https://github.com/new
       Name:        $REPO_NAME
       Visibility:  $VISIBILITY
       Do NOT tick "Add a README", ".gitignore" or "license" — this project
       already has all three, and an auto-created file would collide on push.

  2. Come back here and run:

       git remote add origin git@github.com:$OWNER/$REPO_NAME.git
       git push -u origin main

     (use https://github.com/$OWNER/$REPO_NAME.git instead if you do not have
      SSH keys set up on this machine)

Alternatively install the CLI and re-run this script:

       sudo apt install gh && gh auth login && ./setup_github.sh

EOF
fi

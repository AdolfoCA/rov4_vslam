# AQUA-SLAM tracking statistics: one message per processed stereo frame (/aqua_slam/tracking_info).
# Used to build the "SLAM quality" map: where the ROV was and how well tracking went there.

std_msgs/Header header          # stamp = timestamp of the frame (same as /aqua_slam/orb_pose)

# Tracking::eTrackingState of the frame
int8 STATE_NOT_INITIALIZED = 1
int8 STATE_OK = 2
int8 STATE_RECENTLY_LOST = 3
int8 STATE_LOST = 4
int8 STATE_OK_KLT = 5
int8 STATE_VISUAL_LOST = 6
int8 state

uint32 tracked_map_points       # map points matched in this frame, inliers only (the "Matches" of the info image)
uint32 keypoints                # ORB keypoints of the frame used for tracking (Frame::N)
uint32 map_id                   # id of the active map: it changes when a tracking loss starts a new map
uint32 keyframes_in_map
uint32 map_points_in_map

# Water Linked DVL "position_local" report: the instrument's own dead-reckoned pose.
#
# This is the DVL's internal integration of its bottom-track velocity. It drifts and it
# is NOT a navigation solution - treat it as a reference signal to compare an external
# estimator against, not as ground truth.
#
# The same content is republished as a nav_msgs/Odometry for convenience.

std_msgs/Header header

float64 x                # Dead-reckoned position [m]
float64 y
float64 z
float64 position_std     # Standard deviation of the position estimate [m]

float64 roll             # Attitude as reported by the DVL [rad]
float64 pitch
float64 yaw

uint8 status             # 0 = normal operation
string format            # Report format string as sent by the instrument

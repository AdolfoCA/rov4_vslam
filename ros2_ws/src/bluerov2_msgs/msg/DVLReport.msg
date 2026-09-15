# Complete Water Linked DVL "velocity" report.
#
# The bottom-track velocity is also published on its own as a
# geometry_msgs/TwistWithCovarianceStamped, and the altitude as a sensor_msgs/Range.
# This message keeps everything the instrument sends, including per-beam data and the
# figure of merit, which a SLAM / navigation filter needs in order to reject or
# down-weight individual reports.
#
# All vector quantities have already been rotated from the DVL's native FRD
# (x forward, y right, z down) convention into the ROS FLU convention of `header.frame_id`,
# unless the driver's `rotate_to_flu` parameter was set to false.

std_msgs/Header header

geometry_msgs/Vector3 velocity   # Bottom-track velocity of the DVL frame [m/s]
float64[9] covariance            # Row-major 3x3 velocity covariance [m^2/s^2]
float64 fom                      # Figure of merit: standard deviation of the estimate [m/s]
bool velocity_valid              # False when the DVL has lost bottom lock

float64 altitude                 # Distance to the bottom along the DVL's z axis [m], negative if unknown

DVLBeam[] beams                  # Per-transducer detail

float64 time_delta               # Time span the velocity report covers [s]
uint8 status                     # 0 = normal, 1 = high temperature (instrument derating)
string format                    # Report format string as sent by the instrument

uint64 time_of_validity          # Instrument timestamp of the measurement [us], 0 if unsupported
uint64 time_of_transmission      # Instrument timestamp of transmission [us], 0 if unsupported

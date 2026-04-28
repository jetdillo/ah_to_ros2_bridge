# amazinghand_msgs/msg/FingerCommand.msg
#
# Command for a single finger expressed in semantic axes.
# The hardware interface handles all leader/follower mixing internally.
# Consumers (teleoperation, MoveIt, gesture nodes) never need to know
# about the physical servo topology.

std_msgs/Header header

string  finger_name     # e.g. "index", "middle", "ring", "thumb"
float64 flexion_rad     # + = curl toward palm,  - = extend
float64 abduction_rad   # + = spread away from centre, - = adduct

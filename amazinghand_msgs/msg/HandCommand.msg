# amazinghand_msgs/msg/HandCommand.msg
#
# Command for an entire hand — a variable-length array of FingerCommands.
# Fingers not included in the array retain their current commanded position.
# This design makes it equally valid to command all fingers at once or
# to command a single finger without affecting the others.

std_msgs/Header header
amazinghand_msgs/FingerCommand[] fingers

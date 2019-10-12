# MSG LINUX CPU STATE — 0x7F00 — 32512

std_msgs/UInt8    index   # sequence of this status message, values from 0-9
std_msgs/UInt16   pid     # the PID of the process
std_msgs/UInt8    pcpu    # percent of cpu used, expressed as a fraction of 256
std_msgs/String   tname   # fixed length string representing the thread name
std_msgs/String[] cmdline # the command line (as much as it fits in the remaining packet)

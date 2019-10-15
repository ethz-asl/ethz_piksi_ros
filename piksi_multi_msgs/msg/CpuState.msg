# MSG LINUX CPU STATE — 0x7F00 — 32512

uint8  index   # sequence of this status message, values from 0-9
uint16 pid     # the PID of the process
uint8  pcpu    # percent of cpu used, expressed as a fraction of 256
string tname   # fixed length string representing the thread name
string cmdline # the command line (as much as it fits in the remaining packet)

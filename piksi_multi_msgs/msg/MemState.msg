# MSG LINUX MEM STATE — 0x7F01 — 32513

uint8  index   # sequence of this status message, values from 0-9
uint16 pid     # the PID of the process
uint8  pmem    # percent of memory used, expressed as a fraction of 256
string tname   # fixed length string representing the thread name
string cmdline # the command line (as much as it fits in the remaining packet)

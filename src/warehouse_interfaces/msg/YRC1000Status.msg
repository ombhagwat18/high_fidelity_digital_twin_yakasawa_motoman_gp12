# Simulated YRC1000 controller status — field names/semantics mirror the
# real Yaskawa MotoROS2 status/alarm interface so this is a drop-in swap
# for the real controller later.

uint8 MODE_TEACH  = 0
uint8 MODE_PLAY   = 1
uint8 MODE_REMOTE = 2

bool servo_on
uint8 mode
bool estop
bool alarm_active
uint16 alarm_code
string alarm_text
string job_name
uint32 job_line
uint32 cycle_count

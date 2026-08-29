# Throughput/KPI reporting from warehouse_pick_place's pick_place_logic.

uint32 total
uint32 picked
uint32 failures
uint32 consecutive_failures
float32 last_cycle_time_s
float32 throughput_per_hour   # rolling-window estimate, not naive total/elapsed
string last_bin

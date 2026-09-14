uint8 PENDING=0
uint8 PLANNING=1
uint8 NAVIGATING=2
uint8 WAITING_FOR_STOP=3
uint8 SUCCEEDED=4
uint8 FAILED=5
uint8 CANCELLED=6
uint8 NOT_EXECUTED=7

uint8 index
geometry_msgs/Pose target
uint8 state
bool has_arrival_error
float32 arrival_error_m
float32 elapsed_time_s
uint8 retry_count
string detail

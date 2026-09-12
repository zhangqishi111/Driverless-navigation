uint8 IDLE=0
uint8 ACTIVE=1
uint8 SUCCEEDED=2
uint8 FAILED=3
uint8 CANCELLED=4
uint8 REJECTED=5

std_msgs/Header header
uint32 task_id
bool active
uint8 current_index
uint8 total_points
uint8 state
float32 elapsed_time_s
string detail
landerpi_msgs/NavigationPointState[] points

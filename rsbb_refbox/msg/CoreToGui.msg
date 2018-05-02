uint8 RED = 0
uint8 YELLOW = 1
uint8 NEUTRAL = 2


time clock

string status
string addr
string port

RobotInfo[] active_robots
string[] benchmarking_robots
ZoneState[] zones

string bm_server_status
uint8 bm_server_status_color
string record_server_status
uint8 record_server_status_color

time tablet_last_beacon
bool tablet_display_map
time tablet_call_time
time tablet_position_time
float64 tablet_position_x
float64 tablet_position_y

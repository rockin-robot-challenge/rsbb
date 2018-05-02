uint8 START = 0
uint8 EXECUTING_BENCHMARK = 1
uint8 END = 2
uint8 STOP = 3
uint8 EMERGENCY_STOP = 4
uint8 ERROR = 5
uint8 GLOBAL_TIMEOUT = 6
uint8 READY = 7
uint8 TRANSMITTING_GOAL = 8
uint8 EXECUTING_GOAL = 9
uint8 GOAL_TIMEOUT = 10
uint8 EXECUTING_MANUAL_OPERATION = 11
uint8 NONE = 12

uint8  benchmark_state
string benchmark_payload

uint8  goal_execution_state
string goal_execution_payload

uint8  manual_operation_state
string manual_operation_payload

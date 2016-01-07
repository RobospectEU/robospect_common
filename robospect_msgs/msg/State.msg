# constants
int32 INIT_STATE = 100
int32 STANDBY_STATE = 200
int32 READY_STATE = 300
int32 EMERGENCY_STATE = 400
int32 FAILURE_STATE = 500
int32 SHUTDOWN_STATE = 600
int32 UNKNOWN_STATE = 700
int32 MANUAL_STATE = 800
int32 SPECIAL_STATE = 900
int32 CHARGE_STATE = 1000
int32 PARKING_STATE = 1100

# state of the component
int32 state
# desired control loop frecuency
float32 desired_freq
# real frecuency 
float32 real_freq
# Description of the state
string state_description
# Control mode (POSITION, VELOCITY)
string control_mode

#State of the vehicle+crane system

#Vehicle position+orientation
float32 vehicle_x
float32 vehicle_y
float32 vehicle_theta
#Vehicle velocities
float32 vehicle_linear_speed
float32 vehicle_angular_speed

#Crane position+orientation
float32 crane_x
float32 crane_y
float32 crane_z
float32 crane_q1
float32 crane_q2
float32 crane_q3
float32 crane_q4

#Crane joints
float32[] crane_joints

#Pan&Tilt Angles
float32 pan_angle
float32 tilt_angle

#Vehicle battery level
float32 battery_level

#Vehicle State
#int32 ROBOT_STOPPED = 0
#int32 WAITING_FOR_COMMAND = 1
#int32 VEHICLE_MOVING = 2
#int32 CRANE_MOVING = 3
#int32 FOLDING_CRANE = 4
#int32 state
string state

# Command being executed
string command

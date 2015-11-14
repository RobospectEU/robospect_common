#Mission state from IGC to GCS

#Mission State
string mission_state

#Vehicle State
PlatformState vehicle_state

#Arm State
float32 tip_x
float32 tip_y
float32 tip_z
float32 tip_q1
float32 tip_q2
float32 tip_q3
float32 tip_q4
float32[] arm_joints
string arm_state

#Camera State
string camera_state
string image_file_name
float32 crack_position_x
float32 crack_position_y
float32 crack_position_z
float32 crack_orientation_a
float32 crack_orientation_b
float32 crack_orientation_c

#Ultrasonic State
string ultrasonic_state
float32 crack_width
float32 crack_depth

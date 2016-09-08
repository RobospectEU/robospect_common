#Mission state from IGC to GCS

#Mission State
string mission_state

#Mission Parameters
float32 max_distance
float32 traveled_distance
float32 step_meters
float32 minimum_crack_length
float32 max_cracks
string mission_command
MissionPoint[] points
int16 current_point

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
string image_file_name1
string image_file_name2
string profile_file_name
string stereo_file_name1
string stereo_file_name2
int16 crack_pixel_x
int16 crack_pixel_y
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

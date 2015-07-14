# Robot mission states definition
int32 ROBOT_STOPPED=0				# 0. Robot is stopped
int32 DEFECT_DETECTION=1			# 1. Defect Detection Cameras take snapshot
int32 LASER_SCAN=2				# 3. 3D Laser Scanner takes Measurement
int32 SEND_DISTANCE_FROM_WALL=3			# 4. Robotic Platform sends distance from wall to computer vision system (from robotic laser)
int32 LOCATE_CRACK_POSITION=4			# 5. Computer Vision calculates X Y Z coordinates of crack
int32 MOVE_CRANE_TO_CRACK=5			# 6. Robotic Crane or Tip move closer to crack
int32 CAPTURE_3D_CRACK_SNAPSHOT=6		# 7. Stereo Cameras getting 3D snapshot of crack
int32 MOVE_ARM_TO_CRACK=8			# 8. UltraSonic Sensors Attached to the wall (on crack)
int32 MEASURE_CRACK=9				# 9. UltraSonic Sensors take measurement
int32 CAPTURE_3D_CRACK_SNAPSHOT_OF_SENSOR=10	# 10. Stereo Cameras take stereo-image of sensor onto crack
int32 ROBOT_MOVING_TO_NEXT_POINT=11		# 11. Robot moves to next position and proceeds 1 step*

# Robot general state
State state

# Robot mission state
int32 mission_state

# Robot status
int32 robot_status

# Robot pose (m)
float32 pose_x
float32 pose_y
# (radians)
float32 pose_yaw

# Robot velocity (m/s)
float32 velocity

# Robot battery
float32 battery_voltage
float32 battery_percentage

# Components status?



/*
 * robospect_pad
 * Copyright (c) 2015, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the robot controller, sending the messages received from the joystick device
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <vector>
#include <robotnik_msgs/enable_disable.h>
#include <robotnik_msgs/ptz.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <std_srvs/Empty.h>
#include "robotnik_msgs/set_mode.h"
#include "robotnik_msgs/get_mode.h"

#include <sensor_msgs/JointState.h>
#include <robospect_msgs/SetControlMode.h>
#include <robospect_msgs/PadStatus.h>
#include <robospect_msgs/State.h>
#include <keyboard/Key.h>



#define DEFAULT_MAX_LINEAR_SPEED	    3.0 //m/s
#define DEFAULT_MAX_ANGULAR_POSITION	0.5 // rads/s

#define DEFAULT_MAX_LINEAR_ARM_SPEED	 3.0 //m/s
#define DEFAULT_MAX_ANGULAR_ARM_SPEED	 0.5 // rads/s

#define MAX_NUM_OF_BUTTONS			16
#define MAX_NUM_OF_AXES				8
#define MAX_NUM_OF_BUTTONS_PS3		19
#define MAX_NUM_OF_AXES_PS3			20

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_NUM_OF_AXES			8

#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_ANGULAR		2
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0

#define DEFAULT_JOY					"/joy"
#define DEFAULT_HZ					40.0

#define NUMBER_OF_DRIVEN_JOINTS   	7//

#define DEFAULT_PANTILT_INC			1.0
#define DEFAULT_ZOOM_INC			200.0

#define PA10_COMMAND_UPDOWN			1
#define PA10_COMMAND_LEFTRIGHT		2
#define PA10_COMMAND_FWDBWD			3
#define PA10_COMMAND_YAW			4
#define PA10_COMMAND_PITCH			5
#define PA10_COMMAND_ROLL			6
#define PA10_COMMAND_START			7
#define PA10_COMMAND_STOP			8
#define PA10_COMMAND_SPEED_UP		9
#define PA10_COMMAND_SPEED_DOWN		10

#define DEFAULT_PA10_AXIS_DEADZONE	0.15

//! Class to save the state of the buttons
class Button{
	int iPressed;
	bool bReleased;

	public:

	Button(){
		iPressed = 0;
		bReleased = false;
	}
	//! Set the button as 'pressed'/'released'
	void Press(int value){
		if(iPressed and value==0){
			bReleased = true;
			//ROS_INFO("Released");

		}else if(bReleased and value==1)
			bReleased = false;
			
		iPressed = value;
	}

	int IsPressed(){
		return iPressed;
	}

	bool IsReleased(){
		bool b = bReleased;
		bReleased = false;
		//iPressed = 0;
		return b;
	}
};

class RobospectPad
{
public:
  RobospectPad();

	void ControlLoop();
	int SetStateMode(int state, int arm_mode, int platform_mode);

private:

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void controllerStateCallback(const robospect_msgs::State::ConstPtr& state);

	char * StateToString(int state);
	int SwitchToState(int new_state);

	void PublishState();
	//! Enables/Disables the joystick
	bool EnableDisable(robotnik_msgs::enable_disable::Request &req, robotnik_msgs::enable_disable::Response &res );
	void Update();
	int sendPA10Command(int command_type, int value, bool release);


private:
	ros::NodeHandle nh_, pnh_;

	int axis_linear_speed_, axis_angular_position_;
	
	int axis_pa10_arm_up_down_, axis_pa10_arm_left_right_, axis_pa10_arm_fwd_bwd_;
	int button_pa10_arm_up_, button_pa10_arm_down_, button_pa10_arm_left_, button_pa10_arm_right_;
	int axis_pa10_arm_yaw_, axis_pa10_arm_pitch_, axis_pa10_arm_roll_;
	
	double l_scale_, a_scale_;
	double current_speed_lvl;
	double current_arm_speed_lvl;
	//! Set the max speed sent to the robot
	double max_linear_speed_, max_angular_position_;
	double max_linear_arm_speed_, max_angular_arm_speed_;
	double desired_linear_arm_speed, desired_angular_arm_speed, desired_angular_position, desired_linear_speed;
	//! Desired component's freq
	double desired_freq_;
	//joint index number
	int joint_index;
	//! deadzone when controlling the arm
	double pa10_control_deadzone_;
	//[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0]
	
	robospect_msgs::SetControlMode srv;
	robospect_msgs::State controller_state_msg;

	bool control_crane_enabled;

	// TOPICS
	// subscribers
	//! they will be suscribed to the joysticks
	ros::Subscriber joy_sub_;
	ros::Subscriber controller_state_sub_;
	// publishers
	//! It will publish into command velocity (for the robot)
	ros::Publisher vel_pub_;
	//! Topic to publish the state
	ros::Publisher state_pub_;
	//! Publisher of the arm joint_state
	ros::Publisher joint_pub_;
	//! Publisher of the ptz camera
	ros::Publisher ptz_pub_;
	//! Publish key ops to move the arm
	ros::Publisher keydown_pub_;
	ros::Publisher keyup_pub_;
	
	//! Name of the joint controller topic
	std::string  joint_state_topic;
	//! Controller state topic name
	std::string  controller_state_topic;
	//! PTZ camera control topic name
	std::string  ptz_camera_topic_;
	std::string  keydown_topic_;
	std::string  keydup_topic_;
	//! // Name of the joystick's topic
	std::string  joy_topic_;
	//! Name of the topic where it will be publishing the velocity
	std::string cmd_topic_vel;
	//! topic name for the state
	std::string topic_state_;

	// SERVICES
	//! Service clients
	ros::ServiceServer enable_disable_srv_;
	//! Service to modify the pad control mode
	ros::ServiceClient set_control_mode_srv_;
	//! commands the pa10 arm
	ros::ServiceClient init_platform_srv_;
	ros::ServiceClient reset_encoder_platform_srv_;
	//! Name of the service to change the mode
	std::string set_control_mode_service_name_;

	// JOYSTICK
	//! Current number of buttons of the joystick
	int num_of_buttons_;
	int num_of_axes_;

	//! Vector to save the axis values
	std::vector<float> fAxes;
	//! Vector to save and control the axis values
	std::vector<Button> vButtons;
	//! Number of the DEADMAN button
	int button_dead_man_;
	//! Number of the ARM DEADMAN button
	int button_arm_dead_man_;
	//! Number of the PA10 ARM DEADMAN button
	int button_pa10_arm_dead_man_;
	int button_pa10_arm_euler_dead_man_;
	int button_pa10_arm_start_;
	int button_pa10_arm_stop_;
	int button_pa10_arm_speed_up_;
	int button_pa10_arm_speed_down_;
	//! Number of the deadman button to set the zoom
	int button_zoom_dead_man_;
	//! Number of the button for increase or decrease the speed max of the joystick
	int button_speed_up_, button_speed_down_;
	//! Number of the button for increase or decrease the number of the joint selected
	int button_joint_up_, button_joint_down_;
	//! Number of the button to set the control mode in velocity
	int button_control_mode_velocity_;
	//! Number of the button to set the control mode in velocity
	int button_control_mode_position_;
	//! Degrees of the pan & tilt icreased
	double pantilt_inc_;
	//! zoom increased
	double zoom_inc_;
	//! buttons to the pan-tilt camera
	int ptz_tilt_up_, ptz_tilt_down_, ptz_pan_right_, ptz_pan_left_, ptz_zoom_in_, ptz_zoom_out_;
	//! Name of the service to move ptz
	std::string cmd_service_ptz_;
	

	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq;
	//! Diagnostic to control the reception frequency of the subscribed joy topic
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq;
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_pad;
	//! Diagnostics min freq
	double min_freq_command, min_freq_joy;
	//! Diagnostics max freq
	double max_freq_command, max_freq_joy;
	//! Flag to enable/disable the communication with the publishers topics
	bool bEnable;

	int joint_vector_length;

	std::vector<std::string> JointNames;

	std::string j1_name, j2_name, j3_name, j4_name, j5_name, j6_name;
	std::string j7_name, j8_name, j9_name, j10_name, j11_name, j12_name;
	std::string j13_name, j14_name, j15_name, j16_name, j17_name, j18_name, j19_name, j20_name, j21_name;

};

RobospectPad::RobospectPad():
  axis_linear_speed_(1),
  axis_angular_position_(2),
  pnh_("~")
{
	current_speed_lvl = 0.1;
	joint_index = 0;
	//
	pnh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	pnh_.param("num_of_axes", num_of_axes_, DEFAULT_NUM_OF_AXES);
	pnh_.param("desired_freq", desired_freq_, DEFAULT_HZ);

	if(num_of_axes_ > MAX_NUM_OF_AXES){
		num_of_axes_ = MAX_NUM_OF_AXES;
		ROS_INFO("RobospectPad::RobospectPad: Limiting the max number of axes to %d", MAX_NUM_OF_AXES);
		}
	if(num_of_buttons_ > MAX_NUM_OF_BUTTONS){
		num_of_buttons_ = MAX_NUM_OF_BUTTONS;
		ROS_INFO("RobospectPad::RobospectPad: Limiting the max number of buttons to %d", MAX_NUM_OF_BUTTONS);
		}

	pnh_.param("topic_joy", joy_topic_, std::string(DEFAULT_JOY));


	// MOTION CONF
	pnh_.param("cmd_topic_vel", cmd_topic_vel, std::string("/robospect_robot_control/command"));
	pnh_.param("joint_state_topic", joint_state_topic, std::string ("/robospect_platform_controller/command"));
	pnh_.param("controller_state_topic", controller_state_topic, std::string ("/robospect_platform_controller/state"));
	pnh_.param("ptz_camera_topic", ptz_camera_topic_, std::string ("/axis_v2/ptz_command"));
	pnh_.param("keydown_topic", keydown_topic_, std::string ("keyboard/keydown"));
	pnh_.param("keyup_topic", keydup_topic_, std::string ("keyboard/keyup"));

	pnh_.param("button_arm_dead_man", button_arm_dead_man_, button_arm_dead_man_);
	pnh_.param("button_pa10_arm_dead_man", button_pa10_arm_dead_man_, 9);
	pnh_.param("button_pa10_arm_euler_dead_man", button_pa10_arm_euler_dead_man_, 8);
	pnh_.param("button_pa10_start", button_pa10_arm_start_, 3);
	pnh_.param("button_pa10_stop", button_pa10_arm_stop_, 0);
	pnh_.param("button_pa10_arm_speed_up", button_pa10_arm_speed_up_, 12);
	pnh_.param("button_pa10_arm_speed_down", button_pa10_arm_speed_down_, 14);
	pnh_.param("button_joint_up", button_joint_up_, button_joint_up_);
	pnh_.param("button_joint_down", button_joint_down_, button_joint_down_);
	pnh_.param("button_control_mode_position", button_control_mode_position_, 0);
	pnh_.param("button_control_mode_velocity", button_control_mode_velocity_, 0);

	pnh_.param("button_dead_man", button_dead_man_, button_dead_man_);
	pnh_.param("button_speed_up", button_speed_up_, button_speed_up_);
	pnh_.param("button_speed_down", button_speed_down_, button_speed_down_);
	pnh_.param("max_angular_position", max_angular_position_, DEFAULT_MAX_ANGULAR_POSITION);
	pnh_.param("max_linear_speed_", max_linear_speed_, DEFAULT_MAX_LINEAR_SPEED);
	pnh_.param("max_angular_arm_speed", max_angular_arm_speed_, DEFAULT_MAX_ANGULAR_ARM_SPEED);
	pnh_.param("max_linear_speed_", max_linear_arm_speed_, DEFAULT_MAX_LINEAR_ARM_SPEED);
	pnh_.param("axis_linear_speed", axis_linear_speed_, DEFAULT_AXIS_LINEAR_X);
	pnh_.param("axis_angular_position", axis_angular_position_, DEFAULT_AXIS_ANGULAR);
	pnh_.param("axis_pa10_arm_up_down", axis_pa10_arm_up_down_, axis_pa10_arm_up_down_);
	pnh_.param("axis_pa10_arm_left_right", axis_pa10_arm_left_right_, axis_pa10_arm_left_right_);
	pnh_.param("axis_pa10_arm_fwd_bwd", axis_pa10_arm_fwd_bwd_, axis_pa10_arm_fwd_bwd_);
	pnh_.param("axis_pa10_arm_yaw", axis_pa10_arm_yaw_, axis_pa10_arm_yaw_);
	pnh_.param("axis_pa10_arm_pitch", axis_pa10_arm_pitch_, axis_pa10_arm_pitch_);
	pnh_.param("axis_pa10_arm_roll", axis_pa10_arm_roll_, axis_pa10_arm_roll_);
	pnh_.param("button_pa10_arm_down", button_pa10_arm_down_, 6);
	pnh_.param("button_pa10_arm_up", button_pa10_arm_up_, 4);
	pnh_.param("button_pa10_arm_left", button_pa10_arm_left_, 7);
	pnh_.param("button_pa10_arm_right", button_pa10_arm_right_, 5);
	
	ROS_INFO("axis_linear_speed_ = %d, axis_angular = %d", axis_linear_speed_, axis_angular_position_);
	ROS_INFO("max_linear_speed = %lf, max_angular_speed = %lf", max_linear_speed_, max_angular_position_);

	// DIGITAL OUTPUTS CONF
	pnh_.param("set_control_mode_service_name", set_control_mode_service_name_, std::string("/robospect_platform_controller/set_control_mode"));
	pnh_.param("topic_state", topic_state_, std::string("/robospect_pad/state"));

	// PANTILT CONF
	pnh_.param("cmd_service_ptz", cmd_service_ptz_, cmd_service_ptz_);
	pnh_.param("button_zoom_dead_man", button_zoom_dead_man_, button_zoom_dead_man_);
	pnh_.param("button_ptz_tilt_up", ptz_tilt_up_, ptz_tilt_up_);
	pnh_.param("button_ptz_tilt_down", ptz_tilt_down_, ptz_tilt_down_);
	pnh_.param("button_ptz_pan_right", ptz_pan_right_, ptz_pan_right_);
	pnh_.param("button_ptz_pan_left", ptz_pan_left_, ptz_pan_left_);
	pnh_.param("button_ptz_zoom_in", ptz_zoom_in_, ptz_zoom_in_);
	pnh_.param("button_ptz_zoom_out", ptz_zoom_out_, ptz_zoom_out_);
	pnh_.param("pantilt_inc", pantilt_inc_, DEFAULT_PANTILT_INC);
	pnh_.param("zoom_inc", zoom_inc_, DEFAULT_ZOOM_INC);
	
	
	pnh_.param("pa10_control_deadzone", pa10_control_deadzone_, DEFAULT_PA10_AXIS_DEADZONE);

	// CRANE JOINT names
	pnh_.param("Joints_vector_length", joint_vector_length, 6);
	for(int i= 0; i<joint_vector_length;i++)
		JointNames.push_back("");

	pnh_.param<std::string>("joint1_name", JointNames[0], "crane_first_joint");
	pnh_.param<std::string>("joint2_name", JointNames[1], "crane_second_joint");
	pnh_.param<std::string>("joint3_name", JointNames[2], "crane_third_joint");
	pnh_.param<std::string>("joint4_name", JointNames[3], "crane_fourth_joint");
	pnh_.param<std::string>("joint5_name", JointNames[4], "crane_sixth_joint");
	pnh_.param<std::string>("joint6_name", JointNames[5], "crane_tip_joint");

	ROS_INFO("RobospectPad num_of_buttons_ = %d, axes = %d, topic controller: %s, hz = %.2lf", num_of_buttons_, num_of_axes_, cmd_topic_vel.c_str(), desired_freq_);
	ROS_INFO("RobospectPad: arm deadman button = %d", button_arm_dead_man_);
	ROS_INFO("RobospectPad: pa10 arm deadman button = %d , %d", button_pa10_arm_dead_man_, button_pa10_arm_euler_dead_man_);

	for(int i = 0; i < MAX_NUM_OF_BUTTONS_PS3; i++){
		Button b;
		vButtons.push_back(b);
	}

	for(int i = 0; i < MAX_NUM_OF_AXES_PS3; i++){
		fAxes.push_back(0.0);
	}

	// Publishers
	this->vel_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(this->cmd_topic_vel, 1);
	this->joint_pub_ = nh_.advertise<sensor_msgs::JointState>(this->joint_state_topic, 1);
	this->state_pub_ = pnh_.advertise<robospect_msgs::PadStatus>("state", 1);
	this->ptz_pub_ = nh_.advertise<robotnik_msgs::ptz>(this->ptz_camera_topic_, 10);
	this->keydown_pub_ = nh_.advertise<keyboard::Key>(this->keydown_topic_, 10);
	this->keyup_pub_ = nh_.advertise<keyboard::Key>(this->keydup_topic_, 10);

 	// Listen through the node handle sensor_msgs::Joy messages from joystick
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic_, 10, &RobospectPad::joyCallback, this);
	controller_state_sub_ = nh_.subscribe<robospect_msgs::State>(controller_state_topic, 1, &RobospectPad::controllerStateCallback, this);

 	// 
	set_control_mode_srv_ = nh_.serviceClient<robospect_msgs::SetControlMode>(set_control_mode_service_name_);
	init_platform_srv_ = nh_.serviceClient<std_srvs::Empty>("/robospect_platform_controller/initialize_modbus_controller");
	reset_encoder_platform_srv_ = nh_.serviceClient<std_srvs::Empty>("/robospect_platform_controller/reset_steering_encoder");

	// Diagnostics
	updater_pad.setHardwareID("RobospectPad");
	// Topics freq control
	min_freq_command = min_freq_joy = 5.0;
	max_freq_command = max_freq_joy = 50.0;
	sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

	pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cmd_topic_vel.c_str(), updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));

	// Advertises new service to enable/disable the pad
	enable_disable_srv_ = pnh_.advertiseService("enable_disable",  &RobospectPad::EnableDisable, this);
	//
	bEnable = true;	// Communication flag enabled by default
	control_crane_enabled = false;
    // Info message
    // ROS_INFO("A segfault after this line is usually caused either by bad definition of the pad yaml file or by incrorrect setting of the jsX device in the launch file");
}

/*
 *	\brief Updates the diagnostic component. Diagnostics
 * 		   Publishes the state
 *
 */
void RobospectPad::Update(){
	PublishState();
}

//!
void RobospectPad::PublishState(){
	robospect_msgs::PadStatus pad_state;

	pad_state.platform_mode = controller_state_msg.control_mode;
	
	pad_state.deadman_active = (bool) vButtons[button_dead_man_].IsPressed();
	pad_state.vehicle_speed_level = current_speed_lvl;
	pad_state.desired_angular_position = desired_angular_position;
	pad_state.desired_linear_speed = desired_linear_speed;
	
	pad_state.arm_deadman_active = (bool) vButtons[button_arm_dead_man_].IsPressed();
	pad_state.current_joint = JointNames[joint_index];
	pad_state.arm_speed_level = current_arm_speed_lvl;
	pad_state.current_joint_speed = desired_linear_arm_speed;

	state_pub_.publish(pad_state);
}

/*
 *	\brief Enables/Disables the pad
 */
bool RobospectPad::EnableDisable(robotnik_msgs::enable_disable::Request &req, robotnik_msgs::enable_disable::Response &res )
{
	bEnable = req.value;

	ROS_INFO("RobospectPad::EnablaDisable: Setting to %d", req.value);
	res.ret = true;
	return true;
}

void RobospectPad::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	// First joystick being saved
	for(int i = 0; i < joy->axes.size(); i++){
		this->fAxes[i] = joy->axes[i];
	}
	for(int i = 0; i < joy->buttons.size(); i++){
		this->vButtons[i].Press(joy->buttons[i]);
	}

	//ROS_INFO("RobospectPad::joyCallback: num_of_axes = %d, buttons = %d", (int)(joy->axes.size()), (int)(joy->buttons.size()));
}

void RobospectPad::controllerStateCallback(const robospect_msgs::State::ConstPtr& state)
{
	controller_state_msg = *state;
	
	if(controller_state_msg.control_mode == "VELOCITY")
		control_crane_enabled = true;
	else
		control_crane_enabled = false;
}


int RobospectPad::sendPA10Command(int command_type, int value, bool release){
	keyboard::Key msg;
	msg.header.stamp = ros::Time::now();
	/*
	keyboard::Key::KEY_SPACE
	
	*/
	
	
	switch(command_type){
		case PA10_COMMAND_UPDOWN:
			if(value > 0)
				msg.code = keyboard::Key::KEY_w;
			if(value < 0)
				msg.code = keyboard::Key::KEY_s;
		break;
		case PA10_COMMAND_LEFTRIGHT:
			if(value > 0)
				msg.code = keyboard::Key::KEY_a;
			if(value < 0)
				msg.code = keyboard::Key::KEY_d;
		break;
		case PA10_COMMAND_FWDBWD:
			if(value > 0)
				msg.code = keyboard::Key::KEY_t;
			if(value < 0)
				msg.code = keyboard::Key::KEY_g;
		break;
		case PA10_COMMAND_YAW:
			if(value > 0)
				msg.code = keyboard::Key::KEY_q;
			if(value < 0)
				msg.code = keyboard::Key::KEY_e;
		break;
		case PA10_COMMAND_PITCH:
			if(value > 0)
				msg.code = keyboard::Key::KEY_LEFT;
			if(value < 0)
				msg.code = keyboard::Key::KEY_RIGHT;
		break;
		case PA10_COMMAND_ROLL:
			if(value > 0)
				msg.code = keyboard::Key::KEY_UP;
			if(value < 0)
				msg.code = keyboard::Key::KEY_DOWN;
		break;
		
		case PA10_COMMAND_START:
			msg.code = keyboard::Key::KEY_SPACE;
		break;
		case PA10_COMMAND_STOP:
			msg.code = keyboard::Key::KEY_p;
		break;
		case PA10_COMMAND_SPEED_UP:
			msg.code = keyboard::Key::KEY_o;
		break;
		case PA10_COMMAND_SPEED_DOWN:
			msg.code = keyboard::Key::KEY_k;
		break;
	}
	
	if(release){
		keyup_pub_.publish(msg);
		//usleep(100000);
		//keyup_pub_.publish(msg);
	}else
		keydown_pub_.publish(msg);
		
	
}

//! Controls the actions and states
void RobospectPad::ControlLoop(){

	desired_linear_speed = 0.0;
	desired_angular_position = 0.0;
	desired_linear_arm_speed = 0.0;
	desired_angular_arm_speed = 0.0;
	
	ackermann_msgs::AckermannDriveStamped ref_msg;
	robotnik_msgs::ptz ptz_msg;
	ptz_msg.relative = true;

	ros::Rate r(desired_freq_);

    while(ros::ok()) {

		Update();

		if(bEnable){

			sensor_msgs::JointState joints_msg;

			// MOBILE PLATFORM CONTROL
			if(vButtons[button_dead_man_].IsPressed()){
				
				if(vButtons[button_speed_up_].IsReleased()){
					current_speed_lvl += 0.1;
					if(current_speed_lvl > 1.0)
						current_speed_lvl = 1.0;
				}
				if(vButtons[button_speed_down_].IsReleased()){
					current_speed_lvl -= 0.1;
					if(current_speed_lvl < 0.0)
						current_speed_lvl = 0.0;
				}
				
				ref_msg.header.stamp = ros::Time::now();
				ref_msg.drive.jerk = 0.0;
				ref_msg.drive.acceleration = 0.0;
				ref_msg.drive.steering_angle_velocity = 0.0;

				desired_linear_speed = max_linear_speed_ * current_speed_lvl * fAxes[axis_linear_speed_];
				desired_angular_position = max_angular_position_ * fAxes[axis_angular_position_];

				// ROS_INFO("axis_angular_position_ %d   desired_angular_position=%5.2f", axis_angular_position_, desired_angular_position);

				ref_msg.drive.steering_angle = desired_angular_position;
				ref_msg.drive.speed = desired_linear_speed;

				// Publish into command_vel topic
				vel_pub_.publish(ref_msg);

			}else if(vButtons[button_dead_man_].IsReleased()){
				ref_msg.header.stamp = ros::Time::now();
				ref_msg.drive.jerk = 0.0;
				ref_msg.drive.acceleration = 0.0;
				ref_msg.drive.steering_angle_velocity = 0.0;

				ref_msg.drive.steering_angle = 0.0;
				ref_msg.drive.speed = 0.0;
				//ROS_INFO("RobospectPad::ControlLoop: Deadman released!");
				vel_pub_.publish(ref_msg);// Publish into command_vel topic
			}
			
			if(vButtons[button_pa10_arm_euler_dead_man_].IsPressed()){
				std_srvs::Empty srv_;
				if(vButtons[button_pa10_arm_start_].IsReleased()){
					ROS_INFO("robospect_pad: initializing platform");
					init_platform_srv_.call(srv_);
				}
				if(vButtons[button_pa10_arm_stop_].IsReleased()){
					ROS_INFO("robospect_pad: reseting encoders");
					reset_encoder_platform_srv_.call(srv_);
				}
				
			}
			
			
			// CRANE CONTROL
			/*if(vButtons[button_arm_dead_man_].IsPressed()){
				if(vButtons[button_control_mode_velocity_].IsReleased()){
					srv.request.mode = "VELOCITY";
					if(set_control_mode_srv_.call(srv) != true)
						ROS_ERROR("robospect_pad::ControlLoop: Error calling set control mode service");
				}
				if(vButtons[button_control_mode_position_].IsReleased()){
					srv.request.mode = "POSITION";
					if(set_control_mode_srv_.call(srv) != true)
						ROS_ERROR("robospect_pad::ControlLoop: Error calling set control mode service");
				}
				// Select the arm joint to move
				if(vButtons[button_joint_up_].IsReleased()){
					joint_index += 1;
					if(joint_index >= joint_vector_length)
						joint_index = joint_vector_length -1;
					ROS_INFO("robospect_pad: joint up: %d", joint_index);
				}
				if(vButtons[button_joint_down_].IsReleased()){
					joint_index -= 1;
					if(joint_index < 0)
						joint_index = 0;
					ROS_INFO("robospect_pad: joint down: %d", joint_index);
				}
				if(vButtons[button_speed_up_].IsReleased()){
					current_speed_lvl += 0.1;
					if(current_speed_lvl > 1.0)
						current_speed_lvl = 1.0;
				}
				if(vButtons[button_speed_down_].IsReleased()){
					current_speed_lvl -= 0.1;
					if(current_speed_lvl < 0.0)
						current_speed_lvl = 0.0;
				}
			
				if(control_crane_enabled){
					
					desired_linear_arm_speed = max_linear_arm_speed_ * current_arm_speed_lvl * fAxes[axis_linear_speed_];
					//desired_angular_arm_speed = max_angular_arm_speed_ * current_arm_speed_lvl * fAxes[axis_angular_position_];


					joints_msg.header.stamp = ros::Time::now();
					joints_msg.name.push_back(JointNames[joint_index]);
					joints_msg.velocity.push_back(desired_linear_arm_speed);
					joints_msg.position.push_back(0.0);
					joints_msg.effort.push_back(0.0);
					
					// Publish into command_vel topic
					joint_pub_.publish(joints_msg);

					// speed up or down the arm joints movement
					if(vButtons[button_speed_up_].IsReleased()){
						current_arm_speed_lvl += 0.1;
						if(current_arm_speed_lvl > 1.0)
							current_arm_speed_lvl = 1.0;
					}
					if(vButtons[button_speed_down_].IsReleased()){
						current_arm_speed_lvl -= 0.1;
						if(current_arm_speed_lvl < 0.0)
							current_arm_speed_lvl = 0.0;
					}
							
				}		
			}else if(vButtons[button_arm_dead_man_].IsReleased() and control_crane_enabled){

				joints_msg.header.stamp = ros::Time::now();

				for (size_t i = 0; i < joint_vector_length; i++) {
					joints_msg.name.push_back(JointNames[joint_index]);
					joints_msg.velocity.push_back(0.0);
					joints_msg.position.push_back(0.0);
					joints_msg.effort.push_back(0.0);
				}

				//ROS_INFO("RobospectPad::ControlLoop: Deadman released!");
				joint_pub_.publish(joints_msg);// Publish into command_vel topic
			}
			*/
			//
			// PA10 ARM
			if(vButtons[button_pa10_arm_dead_man_].IsPressed()){
				//ROS_INFO("robospect_pad: start arm");

				if(vButtons[button_pa10_arm_start_].IsReleased()){
					ROS_INFO("robospect_pad: start arm");
					sendPA10Command(PA10_COMMAND_START, 1, false);
				}
				if(vButtons[button_pa10_arm_stop_].IsReleased()){
					sendPA10Command(PA10_COMMAND_STOP, 1, false);
					ROS_INFO("robospect_pad: stop arm");
				}
				if(vButtons[button_pa10_arm_speed_up_].IsReleased()){
					sendPA10Command(PA10_COMMAND_SPEED_UP, 1, false);
					ROS_INFO("robospect_pad: speed up arm");
				}
				if(vButtons[button_pa10_arm_speed_down_].IsReleased()){
					sendPA10Command(PA10_COMMAND_SPEED_DOWN, 1, false);
					ROS_INFO("robospect_pad: speed down arm");
				}
				
				/*
				 * if(vButtons[button_pa10_arm_euler_dead_man_].IsPressed()){
					
					// Stops XYZ cartesian movement
					sendPA10Command(PA10_COMMAND_FWDBWD, 1, true);
					sendPA10Command(PA10_COMMAND_FWDBWD, -1, true);
					sendPA10Command(PA10_COMMAND_LEFTRIGHT, 1, true);
					sendPA10Command(PA10_COMMAND_LEFTRIGHT, -1, true);
					sendPA10Command(PA10_COMMAND_UPDOWN, 1, true);
					sendPA10Command(PA10_COMMAND_UPDOWN, -1, true);
					
					// YAW
					if(fabs(fAxes[axis_pa10_arm_yaw_]) > pa10_control_deadzone_){
						if(fAxes[axis_pa10_arm_yaw_] > 0){
							sendPA10Command(PA10_COMMAND_YAW, -1, false);
						}else{						
							sendPA10Command(PA10_COMMAND_YAW, 1, false);
						}
					}else{
						sendPA10Command(PA10_COMMAND_YAW, 1, true);
						sendPA10Command(PA10_COMMAND_YAW, -1, true);
					}
					// PITCH
					if(fabs(fAxes[axis_pa10_arm_pitch_]) > pa10_control_deadzone_){
						if(fAxes[axis_pa10_arm_pitch_] > 0){
							sendPA10Command(PA10_COMMAND_PITCH, 1, false);
						}else{						
							sendPA10Command(PA10_COMMAND_PITCH, -1, false);
						}
					}else{
						sendPA10Command(PA10_COMMAND_PITCH, 1, true);
						sendPA10Command(PA10_COMMAND_PITCH, -1, true);
					}
					// ROLL
					if(fabs(fAxes[axis_pa10_arm_roll_]) > pa10_control_deadzone_){
						if(fAxes[axis_pa10_arm_roll_] > 0){
							sendPA10Command(PA10_COMMAND_ROLL, -1, false);
						}else{						
							sendPA10Command(PA10_COMMAND_ROLL, 1, false);
						}
					}else{
						sendPA10Command(PA10_COMMAND_ROLL, 1, true);
						sendPA10Command(PA10_COMMAND_ROLL, -1, true);
					}
					
					
				}else if(vButtons[button_pa10_arm_euler_dead_man_].IsReleased()){
					ROS_INFO("PA10 euler deadman released");
					sendPA10Command(PA10_COMMAND_YAW, 1, true);
					sendPA10Command(PA10_COMMAND_YAW, -1, true);
					sendPA10Command(PA10_COMMAND_ROLL, 1, true);
					sendPA10Command(PA10_COMMAND_ROLL, -1, true);
					sendPA10Command(PA10_COMMAND_PITCH, 1, true);
					sendPA10Command(PA10_COMMAND_PITCH, -1, true);
					
				}else{
				* */
				// UP DOWN
				if(vButtons[button_pa10_arm_up_].IsPressed()){
					sendPA10Command(PA10_COMMAND_UPDOWN, 1, false);
				}
				if(vButtons[button_pa10_arm_up_].IsReleased()){
					sendPA10Command(PA10_COMMAND_UPDOWN, 1, true);
				}
				if(vButtons[button_pa10_arm_down_].IsPressed()){
					sendPA10Command(PA10_COMMAND_UPDOWN, -1, false);
				}
				if(vButtons[button_pa10_arm_down_].IsReleased()){
					sendPA10Command(PA10_COMMAND_UPDOWN, -1, true);
				}
				// LEFT RIGHT
				if(vButtons[button_pa10_arm_left_].IsPressed()){
					sendPA10Command(PA10_COMMAND_LEFTRIGHT, 1, false);
				}
				if(vButtons[button_pa10_arm_left_].IsReleased()){
					sendPA10Command(PA10_COMMAND_LEFTRIGHT, 1, true);
				}
				if(vButtons[button_pa10_arm_right_].IsPressed()){
					sendPA10Command(PA10_COMMAND_LEFTRIGHT, -1, false);
				}
				if(vButtons[button_pa10_arm_right_].IsReleased()){
					sendPA10Command(PA10_COMMAND_LEFTRIGHT, -1, true);
				}
				/*
				if(fabs(fAxes[axis_pa10_arm_up_down_]) > pa10_control_deadzone_){
					if(fAxes[axis_pa10_arm_up_down_] > 0){
						//ROS_INFO("updown up");
						sendPA10Command(PA10_COMMAND_UPDOWN, 1, false);
					}else{
						//ROS_INFO("updown down");
						sendPA10Command(PA10_COMMAND_UPDOWN, -1, false);
					}
				}else{
					//ROS_INFO("updown zero");
					sendPA10Command(PA10_COMMAND_UPDOWN, 1, true);
					sendPA10Command(PA10_COMMAND_UPDOWN, -1, true);
				}
				// LEFT RIGHT
				if(fabs(fAxes[axis_pa10_arm_left_right_]) > pa10_control_deadzone_){
					if(fAxes[axis_pa10_arm_left_right_] > 0){
						//ROS_INFO("lefright left");
						sendPA10Command(PA10_COMMAND_LEFTRIGHT, 1, false);

					}else{
						//ROS_INFO("lefright right");
						sendPA10Command(PA10_COMMAND_LEFTRIGHT, -1, false);
					}
				}else{
					//ROS_INFO("lefright zero");
					sendPA10Command(PA10_COMMAND_LEFTRIGHT, 1, true);
					sendPA10Command(PA10_COMMAND_LEFTRIGHT, -1, true);
				}
				// FWD BWD
				if(fabs(fAxes[axis_pa10_arm_fwd_bwd_]) > pa10_control_deadzone_){
					if(fAxes[axis_pa10_arm_fwd_bwd_] > 0){
						//ROS_INFO("fwdbwd fwd");
						sendPA10Command(PA10_COMMAND_FWDBWD, 1, false);
					}else{
						//ROS_INFO("fwdbwd bwd");
						sendPA10Command(PA10_COMMAND_FWDBWD, -1, false);
					}
				}else{
					//ROS_INFO("fwdbwd zero");
					sendPA10Command(PA10_COMMAND_FWDBWD, 1, true);
					sendPA10Command(PA10_COMMAND_FWDBWD, -1, true);
				}
				* */
				// angular vel i,j
				
				if(fabs(fAxes[axis_pa10_arm_fwd_bwd_]) > pa10_control_deadzone_){
					if(fAxes[axis_pa10_arm_fwd_bwd_] > 0){
						sendPA10Command(PA10_COMMAND_FWDBWD, 1, false);
					}else{
						sendPA10Command(PA10_COMMAND_FWDBWD, -1, false);
					}
				}else{
					sendPA10Command(PA10_COMMAND_FWDBWD, 1, true);
					sendPA10Command(PA10_COMMAND_FWDBWD, -1, true);
				}
					
				// YAW
				if(fabs(fAxes[axis_pa10_arm_yaw_]) > pa10_control_deadzone_){
					if(fAxes[axis_pa10_arm_yaw_] > 0){
						sendPA10Command(PA10_COMMAND_YAW, 1, false);
					}else{						
						sendPA10Command(PA10_COMMAND_YAW, -1, false);
					}
				}else{
					sendPA10Command(PA10_COMMAND_YAW, 1, true);
					sendPA10Command(PA10_COMMAND_YAW, -1, true);
				}
				// PITCH
				if(fabs(fAxes[axis_pa10_arm_pitch_]) > pa10_control_deadzone_){
					if(fAxes[axis_pa10_arm_pitch_] > 0){
						sendPA10Command(PA10_COMMAND_PITCH, 1, false);
					}else{						
						sendPA10Command(PA10_COMMAND_PITCH, -1, false);
					}
				}else{
					sendPA10Command(PA10_COMMAND_PITCH, 1, true);
					sendPA10Command(PA10_COMMAND_PITCH, -1, true);
				}
				// ROLL
				if(fabs(fAxes[axis_pa10_arm_roll_]) > pa10_control_deadzone_){
					if(fAxes[axis_pa10_arm_roll_] > 0){
						sendPA10Command(PA10_COMMAND_ROLL, -1, false);
					}else{						
						sendPA10Command(PA10_COMMAND_ROLL, 1, false);
					}
				}else{
					sendPA10Command(PA10_COMMAND_ROLL, 1, true);
					sendPA10Command(PA10_COMMAND_ROLL, -1, true);
				}
				
				

			}else if(vButtons[button_pa10_arm_dead_man_].IsReleased()){
				ROS_INFO("PA10 deadman released");
				/*ROS_INFO("updown zero");
				ROS_INFO("leftright zero");
				ROS_INFO("fwdbwd zero");*/
				sendPA10Command(PA10_COMMAND_FWDBWD, 1, true);
				sendPA10Command(PA10_COMMAND_FWDBWD, -1, true);
				sendPA10Command(PA10_COMMAND_LEFTRIGHT, 1, true);
				sendPA10Command(PA10_COMMAND_LEFTRIGHT, -1, true);
				sendPA10Command(PA10_COMMAND_UPDOWN, 1, true);
				sendPA10Command(PA10_COMMAND_UPDOWN, -1, true);
				
				//ROS_INFO("PA10 euler deadman released");
				//if(vButtons[button_pa10_arm_euler_dead_man_].IsReleased()){
				sendPA10Command(PA10_COMMAND_YAW, 1, true);
				sendPA10Command(PA10_COMMAND_YAW, -1, true);
				sendPA10Command(PA10_COMMAND_ROLL, 1, true);
				sendPA10Command(PA10_COMMAND_ROLL, -1, true);
				sendPA10Command(PA10_COMMAND_PITCH, 1, true);
				sendPA10Command(PA10_COMMAND_PITCH, -1, true);
				//}
			}
			
			// PTZ
			if(not vButtons[button_pa10_arm_dead_man_].IsPressed())
				if( vButtons[button_zoom_dead_man_].IsPressed()){
					if(vButtons[ptz_zoom_in_].IsReleased()){
						//ROS_WARN("robospect_pad: zoom in");
						ptz_msg.tilt = 0.0;
						ptz_msg.pan = 0.0;
						ptz_msg.zoom = zoom_inc_;
						ptz_pub_.publish(ptz_msg);
					}else if(vButtons[ptz_zoom_out_].IsReleased()){
						//ROS_WARN("robospect_pad: zoom out");
						ptz_msg.tilt = 0.0;
						ptz_msg.pan = 0.0;
						ptz_msg.zoom = -zoom_inc_;
						
						ptz_pub_.publish(ptz_msg);
					}
				}else{
					if(vButtons[ptz_tilt_up_].IsReleased()){
						//ROS_WARN("robospect_pad: ptz up");
						ptz_msg.tilt = pantilt_inc_;
						ptz_msg.pan = 0.0;
						ptz_msg.zoom = 0.0;
						ptz_pub_.publish(ptz_msg);
					}
					if(vButtons[ptz_tilt_down_].IsReleased()){
						//ROS_WARN("robospect_pad: ptz down");
						ptz_msg.tilt = -pantilt_inc_;
						ptz_msg.pan = 0.0;
						ptz_msg.zoom = 0.0;
						ptz_pub_.publish(ptz_msg);
					}
					if(vButtons[ptz_pan_right_].IsReleased()){
						//ROS_WARN("robospect_pad: ptz right");
						ptz_msg.tilt = 0.0;
						ptz_msg.pan = pantilt_inc_;
						ptz_msg.zoom = 0.0;
						ptz_pub_.publish(ptz_msg);
					}
					if(vButtons[ptz_pan_left_].IsReleased()){
						//ROS_WARN("robospect_pad: ptz left");
						ptz_msg.tilt = 0.0;
						ptz_msg.pan = -pantilt_inc_;
						ptz_msg.zoom = 0.0;
						ptz_pub_.publish(ptz_msg);
					}
				}
			
			
			
			
		}
		ros::spinOnce();
		r.sleep();
	}
}

///////////////////////// MAIN /////////////////////////////////
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robospect_pad");
	RobospectPad joy;

	joy.ControlLoop();

}

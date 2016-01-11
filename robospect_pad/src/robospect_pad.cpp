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
#include <robotnik_msgs/set_digital_output.h>
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

#define DEFAULT_MAX_LINEAR_SPEED	    3.0 //m/s
#define DEFAULT_MAX_ANGULAR_POSITION	0.5 // rads/s

#define DEFAULT_MAX_LINEAR_ARM_SPEED	    3.0 //m/s
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

#define DEFAULT_JOY			"/joy"
#define DEFAULT_HZ			50.0

#define NUMBER_OF_DRIVEN_JOINTS   7//


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
		if(iPressed and !value){
			bReleased = true;

		}else if(bReleased and value)
			bReleased = false;

		iPressed = value;

	}

	int IsPressed(){
		return iPressed;
	}

	bool IsReleased(){
		bool b = bReleased;
		bReleased = false;
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

	char * StateToString(int state);
	int SwitchToState(int new_state);

	void PublishState();
	//! Enables/Disables the joystick
	bool EnableDisable(robotnik_msgs::enable_disable::Request &req, robotnik_msgs::enable_disable::Response &res );
	void Update();


private:
	ros::NodeHandle nh_, pnh_;

	int axis_linear_speed_, axis_angular_position_;
	double l_scale_, a_scale_;
	double current_speed_lvl;
	double current_arm_speed_lvl;
	//! Set the max speed sent to the robot
	double max_linear_speed_, max_angular_position_;
	double max_linear_arm_speed_, max_angular_arm_speed_;
	//! Desired component's freq
	double desired_freq_;
	//joint index number
	int joint_index;

	// TOPICS
	//! It will publish into command velocity (for the robot)
	ros::Publisher vel_pub_;
	//! they will be suscribed to the joysticks
	ros::Subscriber joy_sub_;
	//! // Name of the joystick's topic
	std::string  joy_topic_;
	//! Name of the topic where it will be publishing the velocity
	std::string cmd_topic_vel;
	//! Name of the service where it will be modifying the digital outputs
	std::string cmd_service_io_;
	//! topic name for the state
	std::string topic_state_;
	//! Topic to publish the state
	ros::Publisher state_pub_;
	//! Publisher of the arm joint_state
	ros::Publisher joint_pub_;
	//! Name of the joint controller topic
	std::string  joint_state_topic;

	// SERVICES
	//! Service clients
	ros::ServiceServer enable_disable_srv_;
	ros::ServiceClient set_digital_outputs_client_;

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
	//! Number of the button for increase or decrease the speed max of the joystick
	int button_speed_up_, button_speed_down_;
	//! Number of the button for increase or decrease the number of the joint selected
	int button_joint_up_, button_joint_down_;

	int output_1_, output_2_;
	bool bOutput1, bOutput2;
	//! buttons to the pan-tilt camera
	int ptz_tilt_up_, ptz_tilt_down_, ptz_pan_right_, ptz_pan_left_;
	//! Name of the service to move ptz
	std::string cmd_service_ptz_;
	//! button to change kinematic mode
	int button_kinematic_mode_;
  	//! kinematic mode
	int kinematic_mode_;
	//! Service to modify the kinematic mode
	ros::ServiceClient setKinematicMode;
	//! Name of the service to change the mode
	std::string cmd_set_mode_;

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

	pnh_.param("button_arm_dead_man", button_arm_dead_man_, button_arm_dead_man_);
	pnh_.param("button_joint_up", button_joint_up_, button_joint_up_);
	pnh_.param("button_joint_down", button_joint_down_, button_joint_down_);

	pnh_.param("button_dead_man", button_dead_man_, button_dead_man_);
	pnh_.param("button_speed_up", button_speed_up_, button_speed_up_);
	pnh_.param("button_speed_down", button_speed_down_, button_speed_down_);
	pnh_.param("max_angular_position", max_angular_position_, DEFAULT_MAX_ANGULAR_POSITION);
	pnh_.param("max_linear_speed_", max_linear_speed_, DEFAULT_MAX_LINEAR_SPEED);
	pnh_.param("max_angular_arm_speed", max_angular_arm_speed_, DEFAULT_MAX_ANGULAR_ARM_SPEED);
	pnh_.param("max_linear_speed_", max_linear_arm_speed_, DEFAULT_MAX_LINEAR_ARM_SPEED);
	pnh_.param("axis_linear_speed", axis_linear_speed_, DEFAULT_AXIS_LINEAR_X);
	pnh_.param("axis_angular_position", axis_angular_position_, DEFAULT_AXIS_ANGULAR);
	ROS_INFO("axis_linear_speed_ = %d, axis_angular = %d", axis_linear_speed_, axis_angular_position_);
	ROS_INFO("max_linear_speed = %lf, max_angular_speed = %lf", max_linear_speed_, max_angular_position_);

	// DIGITAL OUTPUTS CONF
	pnh_.param("cmd_service_io", cmd_service_io_, cmd_service_io_);
	pnh_.param("output_1", output_1_, output_1_);
	pnh_.param("output_2", output_2_, output_2_);
	pnh_.param("topic_state", topic_state_, std::string("/robospect_pad/state"));

	// PANTILT CONF
	pnh_.param("cmd_service_ptz", cmd_service_ptz_, cmd_service_ptz_);
	pnh_.param("button_ptz_tilt_up", ptz_tilt_up_, ptz_tilt_up_);
	pnh_.param("button_ptz_tilt_down", ptz_tilt_down_, ptz_tilt_down_);
	pnh_.param("button_ptz_pan_right", ptz_pan_right_, ptz_pan_right_);
	pnh_.param("button_ptz_pan_left", ptz_pan_left_, ptz_pan_left_);

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

	for(int i = 0; i < MAX_NUM_OF_BUTTONS_PS3; i++){
		Button b;
		vButtons.push_back(b);
	}

	for(int i = 0; i < MAX_NUM_OF_AXES_PS3; i++){
		fAxes.push_back(0.0);
	}


  this->vel_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(this->cmd_topic_vel, 1);

	this->joint_pub_ = nh_.advertise<sensor_msgs::JointState>(this->joint_state_topic, 1);

 	// Listen through the node handle sensor_msgs::Joy messages from joystick
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic_, 1, &RobospectPad::joyCallback, this);

 	// Request service to activate / deactivate digital I/O
	set_digital_outputs_client_ = nh_.serviceClient<robotnik_msgs::set_digital_output>(cmd_service_io_);

	bOutput1 = bOutput2 = false;

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
	/*RobospectPad::rescuer_pad_state pad_state;

	pad_state.state = StateToString(iState);
	pad_state.arm_mode = ModeToString(iArmMode);
	pad_state.platform_mode = ModeToString(iPlatformMode);
	pad_state.speed_level = current_speed_lvl;
	pad_state.deadman_active = (bool) vButtons[button_dead_man_].IsPressed();

	state_pub_.publish(pad_state);*/
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

//! Controls the actions and states
void RobospectPad::ControlLoop(){

	double desired_linear_speed = 0.0, desired_angular_position = 0.0;
	ackermann_msgs::AckermannDriveStamped ref_msg;

	double desired_linear_arm_speed = 0.0, desired_angular_arm_speed = 0.0;

	ros::Rate r(desired_freq_);

    while(ros::ok()) {

		Update();

		if(bEnable){

			sensor_msgs::JointState joints_msg;

			if(vButtons[button_dead_man_].IsPressed()){
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


			if(vButtons[button_arm_dead_man_].IsPressed()){
				desired_linear_arm_speed = max_linear_arm_speed_ * current_arm_speed_lvl * fAxes[axis_linear_speed_];
				//desired_angular_arm_speed = max_angular_arm_speed_ * current_arm_speed_lvl * fAxes[axis_angular_position_];


				joints_msg.header.stamp = ros::Time::now();
				joints_msg.name.push_back(JointNames[joint_index]);
				joints_msg.velocity.push_back(desired_linear_arm_speed);
				joints_msg.position.push_back(0.0);
				joints_msg.effort.push_back(0.0);
				/*
				if (joint_index == 0 || joint_index == 3 || joint_index == 7 ) {  //different axis for some joints
					joints_msg.velocity.push_back(desired_angular_arm_speed);
				}else{
					joints_msg.velocity.push_back(desired_linear_arm_speed);
				}
				*/
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
				// Select the arm joint to move
				if(vButtons[button_joint_up_].IsReleased()){
					joint_index += 1;
					if(joint_index >= joint_vector_length)
						joint_index = joint_vector_length -1;
				}
				if(vButtons[button_joint_down_].IsReleased()){
					joint_index -= 1;
					if(joint_index < 0)
						joint_index = 0;
				}

			}else if(vButtons[button_dead_man_].IsReleased()){

				joints_msg.header.stamp = ros::Time::now();

				for (size_t i = 0; i < joint_vector_length; i++) {
					joints_msg.name.push_back(JointNames[joint_index]);
					joints_msg.velocity.push_back(desired_linear_arm_speed);
					joints_msg.position.push_back(0.0);
					joints_msg.effort.push_back(0.0);
				}

				//ROS_INFO("RobospectPad::ControlLoop: Deadman released!");
				joint_pub_.publish(joints_msg);// Publish into command_vel topic
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

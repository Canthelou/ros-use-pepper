/**
* \file sensor_return.cpp
* \brief Allow a simulation of a autonomous run on Pepper
* \author Vivien.C
* \date 02/05/2018
*
* Movement programme for Pepper
* Input the information from 3 differents topics during all it's run
* Publish goals on the movement topic
*
*/


#include <ros/ros.h>		/* Ros library */
#include <std_msgs/Int8.h> 	/* Topic type */
#include <std_msgs/Bool.h>	/* Topic type */
#include <geometry_msgs/Twist.h>  	/* Topic cmd_vel type */
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <geometry_msgs/PoseStamped.h> 	/* Topic move_base/goal type */
#include <time.h>       	/* time */

class autoWalking
{
public:
	autoWalking() //Constructor
	{
		//Publisher
		pubGoal_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
		pubArm_ = n_.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 10);

		//Subscriber
		subHead_ = n_.subscribe("/demo_control/head_touch", 1, &autoWalking::TouchHead, this);
		pubSonarFront_ = n_.subscribe("/demo_control/sonar/front", 1, &autoWalking::FrontDistance, this);
		pubSonarBack_ = n_.subscribe("/demo_control/sonar/back", 1, &autoWalking::BackDistance, this);

		//Timer
		walkingTimer = n_.createTimer(ros::Duration(0.1),&autoWalking::WalkingCallBack, this);
		walkingMotion = n_.createTimer(ros::Duration(1.0),&autoWalking::WalkingMotionCallBack, this);
		rotateTimer = n_.createTimer(ros::Duration(5.0),&autoWalking::RotateCallBack, this);

		//Prgm Var
		moveForward = false; moveBackward = false; 
		obstacleFront = false, obstacleBack = false;
		findBetterWay = false; endRotate = false;
		moveArm = false;


		armControl.speed = 0.1;
		headControl.speed = 0.1;
		shoulderControl.speed = 0.1;
		armControl.joint_names.push_back("RShoulderPitch");
		armControl.joint_names.push_back("LShoulderPitch");
		shoulderControl.joint_names.push_back("RShoulderRoll");
		shoulderControl.joint_names.push_back("LShoulderRoll");
		headControl.joint_names.push_back("HeadYaw");
		headControl.joint_names.push_back("HeadPitch");
		armControl.joint_angles.push_back(0);
    	armControl.joint_angles.push_back(0);
		shoulderControl.joint_angles.push_back(0);
		shoulderControl.joint_angles.push_back(0);
		headControl.joint_angles.push_back(0);
		headControl.joint_angles.push_back(0);
		pubArm_.publish(shoulderControl);

		movement.header.frame_id = "base_footprint";
		movement.pose.position.x = 0.0;
		movement.pose.position.y = 0.0;
		movement.pose.position.z = 0.0;
		movement.pose.orientation.x = 0.0;
		movement.pose.orientation.y = 0.0;
		movement.pose.orientation.z = 0.0;
		movement.pose.orientation.w = 1.0;

		direction = 0;
		srand (time(NULL));
	}


	/** Subcriber CallBack **/

	/**
	* \fn void TouchHead(const std_msgs::Int8 &msg)
	* \brief Allow to have a sort of HIM on the head of Pepper
	*
	* \param msg Input message from the Topic /demo_control/head_touch
	* \return void
	*/
	void TouchHead(const std_msgs::Int8 &msg)
	{
		ROS_INFO("I heard head touch: %d",msg.data);
		switch (msg.data)
		{
		case 0://Front Sensor -> Forward while no obs
			moveForward = true;
			break;
		case 1://Middle Sensor -> Stop
			StopMovement();
			break;
		case 2://Back Sensor -> Backward 1 meter if no obs
			moveBackward = true;
			break;
		}
	}

	/**
	* \fn void FrontDistance(const std_msgs::Bool &msg){
	* \brief Recup the front distance on the sonar
	*
	* \param msg Input message from the Topic /demo_control/sonar/front
	* \return void
	*/
	void FrontDistance(const std_msgs::Bool &msg){
		obstacleFront = msg.data; //If obstacle, it ask to stop moving
	}

	/**
	* \fn void BackDistance(const std_msgs::Bool &msg){
	* \brief Recup the back distance on the sonar
	*
	* \param msg Input message from the Topic /demo_control/sonar/back
	* \return void
	*/
	void BackDistance(const std_msgs::Bool &msg){
		obstacleBack = msg.data; //If obstacle, it ask to not moving
	}


	/** Timer CallBack **/

	/**
	* \fn void WalkingCallBack(const ros::TimerEvent& event){
	* \brief Gesture of the possible movement, call each 0.1 seconde
	*
	* Go forward/backward if and while there are no obstacles
	* Ask to find a new way when it's blocked
	*
	* \param event Necessary for the timer
	* \return void
	*/
	void WalkingCallBack(const ros::TimerEvent& event){
		if (moveForward && obstacleFront){ 
			//Sort of autonomous life
			StopMovement();
			ROS_INFO("Still alive");
			findBetterWay = true;
		}else{
			if (moveForward && !obstacleFront)
				GoForward();
			else{
				if (moveBackward && obstacleBack)
					ROS_INFO("Can't go backward");
				else{
					if (moveBackward && !obstacleBack)
						GoBackward();
				}
			}
		}
	}

	/**
	* \fn void WalkingMotionCallBack(const ros::TimerEvent& event){
	* \brief Allow the motion of the head and arm
	*
	* When it go forward, arms move alternately
	* When it search a new way, the head is facing the direction
	*
	* \param event Necessary for the timer
	* \return void
	*/
	void WalkingMotionCallBack(const ros::TimerEvent& event){
		//Allows the natural movement of the arms
		//Right Arm
		if (moveForward && obstacleFront){
			ROS_INFO("stop arm moving");
			//Arm back to normal
			armControl.joint_angles[0] = 100*M_PI/180; 	//Right Arm
    		armControl.joint_angles[1] = 100*M_PI/180;	//Left Arm
			pubArm_.publish(armControl);
		}else{
			if (moveForward && !obstacleFront){
				ROS_INFO("arm move");
				//Allows the natural movement of the arms
				//Right Arm
				if(moveArm){
					//Right Arm
					armControl.joint_angles[0] = 119.5*M_PI/180;
					//Left Arm
					armControl.joint_angles[1] = 70*M_PI/180;
				}else{
					armControl.joint_angles[1] = 119.5*M_PI/180;
					armControl.joint_angles[0] = 70*M_PI/180;
				}
				moveArm = (moveArm)? false:true;
				pubArm_.publish(armControl);
			}
		}
	}
	
	/**
	* \fn void RotateCallBack(const ros::TimerEvent& event){
	* \brief Find the better way to go after a stop 
	*
	* evaluate obstacle at Left and Right with a cooldown of 10 secondes
	* waiting until the rotation is finished
	*
	* \param event Necessary for the timer
	* \return void
	*/
	void RotateCallBack(const ros::TimerEvent& event){
	    if(findBetterWay){
		//Pepper is facing an obstacle and want to go an other way
			armControl.joint_angles[0] = 100*M_PI/180; 	//Right Arm
    		armControl.joint_angles[1] = 100*M_PI/180;	//Left Arm
			shoulderControl.joint_angles[0] = 0;
			shoulderControl.joint_angles[1] = 0;
			pubArm_.publish(armControl); pubArm_.publish(shoulderControl);
			if(!endRotate){
				//First rotation
				ROS_INFO("Rotate and waiting");
				direction = rand() % 100 + 1;
				ROS_INFO("%d", direction);
				movement.pose.orientation.z = (direction < 50)? 1.0:-1.0;//random direction
				pubGoal_.publish(movement);
				endRotate = true;
				
				//Head movement
				headControl.joint_angles[0] = (direction > 50)? -1*30*M_PI/180 : 30*M_PI/180;
				headControl.joint_angles[1] = 10*M_PI/180;
				pubArm_.publish(headControl);
			} else{
				//Pepper is facing a new way, is there an obstacle ?
				if(endRotate && !obstacleFront){ //No obs
					//Pepper facing the new way, go forward
					moveForward = true;
					endRotate = false;
					findBetterWay = false;

					headControl.joint_angles[0] = 0;
					headControl.joint_angles[1] = 0;
					pubArm_.publish(headControl);
				} else{ //Yes an obs
					//Second rotation
					ROS_INFO("Rotate and waiting");
					movement.pose.orientation.z = (direction < 50)? -2.0:2.0; //Opposite direction
					pubGoal_.publish(movement);			

					//Head rotation
					headControl.joint_angles[0] = -1.0 * 30*M_PI/180;
					pubArm_.publish(headControl);
				}
			}
	    }
	}

	/**
	* \fn void StopMovement(){
	* \brief Initialise and allow the programme to stop the movement
	*
	* \return void
	*/
	void StopMovement(){
		movement.pose.position.x = 0.0;
		movement.pose.orientation.z = 0.0;
		movement.pose.orientation.w = 1.0;
		ROS_INFO("Stop moving");
		pubGoal_.publish(movement);
		moveForward = false; moveBackward = false;
		endRotate = false; findBetterWay = false;
	}

	/**
	* \fn void GoForward(){
	* \brief Initialise and allow the programme to go forward
	*
	* \return void
	*/
	void GoForward(){
		movement.pose.position.x = 0.0;
		movement.pose.orientation.z = 0.0;
		movement.pose.orientation.w = 1.0;
		movement.pose.position.x = 10.0; //Big goal
		ROS_INFO("Forward while no obs...");
		pubGoal_.publish(movement);
	}

	/**
	* \fn void GoBackward(){
	* \brief Initialise and allow the programme to go backward
	*
	* \return void
	*/
	void GoBackward(){
		movement.pose.position.x = 0.0;
		movement.pose.orientation.z = 0.0;
		movement.pose.orientation.w = 1.0;
		movement.pose.position.x = -1.0; //-1 meter back
		ROS_INFO("1 meter backward.");
		pubGoal_.publish(movement);
	    	moveBackward = false;
	}
 	

private:
	//Subs/Pub
	ros::NodeHandle n_;
	ros::Publisher pubGoal_, pubArm_;
	ros::Subscriber subHead_, pubSonarFront_, pubSonarBack_;

	//Timer
	ros::Timer walkingTimer, walkingMotion, rotateTimer;

	//Var
	geometry_msgs::PoseStamped movement;
	naoqi_bridge_msgs::JointAnglesWithSpeed armControl, headControl, shoulderControl;
	int direction;
	float angleValue[];
	bool findBetterWay, endRotate, moveBackward, moveForward, obstacleFront, obstacleBack, moveArm;
};//End of class autoWalking


int main(int argc, char **argv)
{
	ROS_INFO("Ready to go !");
	ros::init(argc, argv, "subscribe_and_publish");

	autoWalking myObject;

	ros::spin();

	return 0;
}

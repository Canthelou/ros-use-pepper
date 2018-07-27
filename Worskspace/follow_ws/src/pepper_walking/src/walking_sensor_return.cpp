/**
* \file walking_autonomous.cpp
* \brief Subcribe, analyse and convert all the data of the sonar and head topics
* \author Vivien.C
* \date 02/05/2018
*
* Convert distance from sonar to a simple data
* Use head of Pepper like an interface with human
* Say somes things during some actions or evenements
*
*/


#include <ros/ros.h>
#include <string>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

class sensorInfo
{
public:
	sensorInfo() //Constructor
	{
		//Publisher
		pubSpeech_ = n_.advertise<std_msgs::String>("/speech", 1);
		//create 3 topics, one for the head, two for the sonars
		pubSonarFront_ = n_.advertise<std_msgs::Bool>("/demo_control/sonar/front", 1000);
		pubSonarBack_ = n_.advertise<std_msgs::Bool>("/demo_control/sonar/back", 1000);
		pubTouchHead_ = n_.advertise<std_msgs::Int8>("/demo_control/head_touch", 1000);

		//Subscriber
		subHead_ = n_.subscribe("/pepper_robot/naoqi_driver/head_touch", 1, &sensorInfo::touchHead, this);
		subSonarFront_ = n_.subscribe("/pepper_robot/naoqi_driver/sonar/front", 1, &sensorInfo::sonarFront, this);
		subSonarBack_ = n_.subscribe("/pepper_robot/naoqi_driver/sonar/back", 1, &sensorInfo::sonarBack, this);

		//Timer
		sonarFrontTimer = n_.createTimer(ros::Duration(3), &sensorInfo::sonarFrontCallBack, this);
		sonarBackTimer = n_.createTimer(ros::Duration(3), &sensorInfo::sonarBackCallBack, this);
		headTouchTimer = n_.createTimer(ros::Duration(1), &sensorInfo::headTouchCallBack, this);

		//Prgm Var
		headButton.data = -1;
		infOneMeterFront.data = false; infOneMeterBack.data = false; headSensorBool.data = false;
		distanceFront = 0.0; distanceBack = 0.0;
	}

	/** Subcriber CallBack **/

	/**
	* \fn void touchHead(const naoqi_bridge_msgs::HeadTouch &msg)
	* \brief Allow to have a sort of HIM on the head of Pepper
	*
	* \param msg Input message from the Topic /pepper_robot/naoqi_driver/head_touch
	* \return void
	*/
	void touchHead(const naoqi_bridge_msgs::HeadTouch &msg)
	{
		headSensorBool.data = (msg.state != 0) ? true : false;
		headButton.data = msg.button;
		if (headSensorBool.data) pubTouchHead_.publish(headButton);
	}

	/**
	* \fn void sonarFront(const sensor_msgs::Range &msg)
	* \brief Recup the front distance on the sonar
	*
	* \param msg Input message from the Topic /pepper_robot/naoqi_driver/sonar/front
	* \return void
	*/
	void sonarFront(const sensor_msgs::Range &msg)
	{
		infOneMeterFront.data = (msg.range <= 1.0) ? true : false;
		distanceFront = msg.range;
		pubSonarFront_.publish(infOneMeterFront);
	}

	/**
	* \fn void sonarBack(const sensor_msgs::Range &msg)
	* \brief Recup the back distance on the sonar
	*
	* \param msg Input message from the Topic /pepper_robot/naoqi_driver/sonar/back
	* \return void
	*/
	void sonarBack(const sensor_msgs::Range &msg)
	{
		infOneMeterBack.data = (msg.range <= 1.0) ? true : false;
		distanceBack = msg.range;
		pubSonarBack_.publish(infOneMeterBack);
	}

	/** Timer CallBack **/
	
	/**
	* \fn void headTouchCallBack(const ros::TimerEvent& event)
	* \brief Avoid to have too much messages on the pepper topic /speech
	*
	* Exe every seconde
	* Say something when the head is touch
	*
	* \param event Necessary for the timer
	* \return void
	*/
	void headTouchCallBack(const ros::TimerEvent& event)
	{
		std::string text;
		if (headSensorBool.data) {
			//Say and write area touch
			text.append((headButton.data == 0) ? "En avant !" :
				(headButton.data == 1) ? "Stoppez les machines !" : "Attention derrière !");
			ROS_INFO("Head sensor: %d", headButton.data);

			//Publish it on Pepper
			std_msgs::String output;
			output.data = text;
			pubSpeech_.publish(output);
		}
	}

	/**
	* \fn void headTouchCallBack(const ros::TimerEvent& event)
	* \brief Avoid to have too much messages on the pepper topic /speech
	*
	* Exe every 3 seconde
	* Say something if there's an obstacle
	*
	* \param event Necessary for the timer
	* \return void
	*/
	void sonarFrontCallBack(const ros::TimerEvent& event)
	{
		if (infOneMeterFront.data) {
			std::string text;
			ROS_INFO("Front distance: %f", distanceFront); //Console
			text.append("schtroumpf devant"); //Pepper
			std_msgs::String output;
			output.data = text;
			//pubSpeech_.publish(output); //Of for test
		}
	}

	/**
	* \fn void headTouchCallBack(const ros::TimerEvent& event)
	* \brief Avoid to have too much messages on the pepper topic /speech
	*
	* Exe every 3 seconde
	* Say something if there's an obstacle
	*
	* \param event Necessary for the timer
	* \return void
	*/
	void sonarBackCallBack(const ros::TimerEvent& event)
	{
		if (infOneMeterBack.data) {
			std::string text;
			ROS_INFO("Back distance: %f", distanceBack); //Console
			text.append("schtroumpf derrière"); //Pepper
			std_msgs::String output;
			output.data = text;
			//pubSpeech_.publish(output); //Of for test
		}
	}

private:
	//Subs/Pub
	ros::NodeHandle n_;
	ros::Publisher pubSpeech_, pubSonarFront_, pubSonarBack_, pubTouchHead_;
	ros::Subscriber subHead_, subSonarFront_, subSonarBack_;

	//Timer
	ros::Timer sonarFrontTimer, sonarBackTimer, headTouchTimer;

	//Var
	std_msgs::Int8 headButton;
	std_msgs::Bool infOneMeterFront, infOneMeterBack, headSensorBool;
	float distanceFront, distanceBack;

};//End of class sensorInfo


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensor_info");

	sensorInfo myObject;

	ros::spin();

	return 0;
}

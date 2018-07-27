/**
* \brief Exemple of speech publisher on Pepper
* \author Vivien.C
* \date 18/07/2018
*
* Pepper react when touching it head or hand
* There are 3 area on it head, front, top and back
* On it hand there is just one
* It say something different for each area
*
*/

#include <ros/ros.h>
#include <string>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <std_msgs/String.h>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Speeching pepper publisher 
		pubSpeech_ = n_.advertise<std_msgs::String>("/speech", 1);

		//Head_touching subscriber
		subHead_ = n_.subscribe("/pepper_robot/naoqi_driver/head_touch", 1, &SubscribeAndPublish::touchHead, this);
		subHand_ = n_.subscribe("/pepper_robot/naoqi_driver/hand_touch", 1, &SubscribeAndPublish::touchHand, this);
	}

	/**
	* \fn void touchHead(const naoqi_bridge_msgs::HeadTouch &msg)
	* \brief Allow to have a sort of HIM on the head of Pepper
	*
	* \param msg Input message from the Topic /demo_control/head_touch
	* \return void
	*/
	void touchHead(const naoqi_bridge_msgs::HeadTouch &msg)
	{
		std::string text;
		//Test on the state of the buttons 
		if(msg.state!=0){
			//Creating the text that pepper going to repeat
			text.append("on me touche ");
			text.append((msg.button == 0)? "le front":(msg.button == 1)? "la tête": "l'arrière du crane");

			ROS_INFO("Head sensor: %d     %s", msg.button,text.c_str());

			std_msgs::String output;
			output.data = text;
			pubSpeech_.publish(output);
		}
	}

	void touchHand(const naoqi_bridge_msgs::HandTouch &msg)
	{
		std::string text;
		//Test on the state of the buttons 
    	if(msg.state!=0){
			//Creating the text that pepper going to repeat
			text.append("on me serre ");
			text.append((msg.hand == 0)? "la main droite":(msg.hand == 3)? "la main gauche ": "quelque chose");

			ROS_INFO("Hand sensor: %d     %s", msg.hand,text.c_str());

			std_msgs::String output;
			output.data = text;
			pubSpeech_.publish(output);
		}
	}

 private:
  ros::NodeHandle n_; 
  ros::Publisher pubSpeech_;
  ros::Subscriber subHead_;
  ros::Subscriber subHand_;

};//End of class SubscribeAndPublish


int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_publish");

  SubscribeAndPublish myObject;

  ros::spin();

  return 0;
}

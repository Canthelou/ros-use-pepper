/**
* \brief Allowing to control Pepper with the keyboard
* \author Vivien.C
* \date 18/07/2018
*
* Movement programme (/cmd_vel) for Pepper
* Publish goals on the movement topic
*
*/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <iostream>

/*void talkBack(const sensor_msgs::LaserScan &back){
   //ROS_INFO(back.header);
}*/


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_square");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  //ros::Subscriber sub = n.subscribe("pepper_robot/naoqi_driver/laser", 1000, talkBack);

  ros::Rate rate(1);

   bool command_known,stop = false;


  std::cout << "Type a command and then press enter."
      "Use ZQSD to move lineary Pepper\n"
      "Or use AE to turn him.\n"
      "'.' to stop.\n";

  while (ros::ok() && !stop)
  {
    geometry_msgs::Twist msg;

    std::cout << "\nUse 'ZQSDAE' to control Pepper, space to stop:\n";

    char cmd[50]; //Just to make sure it's long enought
   
    std::cin.getline(cmd, 50);

	//Test the letter
    command_known = true; //Know this letter ?
    switch(cmd[0]){
	case 'z': //Forward
	  msg.linear.x = 0.05;
	  ROS_INFO("Forward %f",msg.linear.x);
	break;
	case 's': //Backward
	  msg.linear.x = -0.05;
 	  ROS_INFO("Backward %f", msg.linear.x);
	break;
	case 'q': //Left
	  msg.linear.y = 0.05;
	  ROS_INFO("Left %f", msg.linear.y);
	break;
	case 'd': //Right
	  msg.linear.y = -0.05;
	  ROS_INFO("Right %f", msg.linear.y);
	break;
	case 'a': //Rotate Left
	  msg.angular.z = 0.2;
	  ROS_INFO("Rotate Left %f", msg.angular.z);
	break;
	case 'e': //Rotate Right
	  msg.angular.z = -0.2;
	  ROS_INFO("Rotate Right %f", msg.angular.z);
	break;
	case ' ': //Stop
	  msg.linear.x = 0; msg.linear.y = 0; msg.angular.z = 0; //Reset movement
	  ROS_INFO("Stop");
	break;
	default:
	  std::cout << "Unknown Command:"<<cmd<<" !\n";
  	  command_known = false;
	break;
    }
    if(command_known){
	chatter_pub.publish(msg);
	rate.sleep();
    }
  }
  return 0;
}

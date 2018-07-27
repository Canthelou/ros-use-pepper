/**
* \brief Allowing to control Pepper with the keyboard
* \author Vivien.C
* \date 18/07/2018
*
* Pose movement programme for Pepper
* Publish goals on the movement topic
*
*/

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle n; 

  //Publisher
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  //Rate
  ros::Rate loop_rate(10);
  

  ROS_INFO("Ready to go !");
  
  geometry_msgs::PoseStamped msg;  

   /*//we'll send a goal to the robot to move 1 meter forward
   msg.header.stamp = ros::Time::now(); 
   msg.header.frame_id = "base_footprint"; 
   msg.pose.position.x = 1.0;
   msg.pose.position.y = 0.0;
   msg.pose.position.z = 0.0;
   msg.pose.orientation.x = 0.0;
   msg.pose.orientation.y = 0.0;
   msg.pose.orientation.z = 0.0;
   msg.pose.orientation.w = 1.0;*/


   bool command_known, stop = false;
   while (ros::ok() && !stop)
   {
      std::cout << "\nUse 'ZQSD' to control Pepper; 'SpaceBar' to stop:\n";
      char cmd[50]; //Just to make sure it's long enought
      std::cin.getline(cmd, 50);

	  //Initialize the movement
      msg.header.frame_id = "base_footprint"; 
      msg.pose.position.x = 0.0;
      msg.pose.position.y = 0.0;
      msg.pose.position.z = 0.0;
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 1.0;
      
	  //Test the letter
      switch(cmd[0]){
    case 'z': //Forward
      msg.pose.position.x = 1.0;
      ROS_INFO("Forward");
    break;
    case 's': //Backward
      msg.pose.position.x = -1.0;
      ROS_INFO("Backward");
    break;
    case 'q': //Left
      msg.pose.position.x = 0.5;
      msg.pose.orientation.z = 1.0;
      ROS_INFO("Left");
    break;
    case 'd': //Right
      msg.pose.position.x = 0.5;
      msg.pose.orientation.z = -1.0;
      ROS_INFO("Right");
    break;
    case ' ': //Stop
      goal_pub.publish(msg);
      ROS_INFO("Stop");
    break;
    default:
      std::cout <<"Unknown Command: "<<cmd<<" !\n";
    break;
      }

      if(msg.pose.position.x != 0){
			goal_pub.publish(msg);
			loop_rate.sleep();
      }
   }
  return 0; 
}

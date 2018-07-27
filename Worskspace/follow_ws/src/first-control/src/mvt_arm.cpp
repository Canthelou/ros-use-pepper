/**
* \brief Allow to move the right arm of pepper
* \author Vivien.C
* \date 18/07/2018
*
* Movement programme (/cmd_vel) for Pepper
* Input the information from 3 differents topics during all it's run
* Publish goals on the movement topic
*
*/

#include <ros/ros.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <math.h>
#include <stdio.h>


int main(int argc, char **argv)
{
    float angleValue[] = {
        5
        ,45
        ,-90
        ,0

        ,100
        ,0.5
        ,0
        ,75
    }; //Value for movement

    ros::init(argc, argv, "publisher_arm");
    ros::NodeHandle n_;

    //Publisher
    ros::Publisher pubArm_ = n_.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 10);

    ros::Rate loop_rate(1);

	//Initialize the movement
    naoqi_bridge_msgs::JointAnglesWithSpeed jointControl;
    jointControl.speed = 0.2;
    jointControl.joint_names.push_back("RShoulderPitch");
    jointControl.joint_names.push_back("RElbowRoll");
    jointControl.joint_names.push_back("RWristYaw");
    jointControl.joint_names.push_back("RHand");
    jointControl.joint_angles.push_back(0);
    jointControl.joint_angles.push_back(0);
    jointControl.joint_angles.push_back(0);
    jointControl.joint_angles.push_back(0);

    //Avoid losing time to change simple var
    std::cout << "\nEnter special value ? (y/n): ";
    char cmd1[10]; //Just to make sure it's long enought
    std::cin.getline(cmd1, 10);

    switch(cmd1[0]){
            case 'y'://Want a change
                std::cout << "\nEnter up value:\nShoulderPitch (-119.5° to 119.5°) : ";
                std::cin >> angleValue[0];                      //Arm
                std::cout << "ElbowRoll (0.5° to 89.5°) : ";
                std::cin >> angleValue[1];                      //Elbow
                std::cout << "WristYaw (-104.5° to 104.5°) : ";
                std::cin >> angleValue[2];                      //Wrist
                std::cout << "Hand (0 to 100%) : ";
                std::cin >> angleValue[3];                      //Hand
            break;
    }   

    while (ros::ok())
    {
        std::cout << "\nUse z or s to move arm:   ";
        char cmd[10]; //Just to make sure it's long enought
        std::cin.getline(cmd, 10);

        switch(cmd[0]){
            case 'z': //Arm up
                ROS_INFO("Arm going up");
                jointControl.joint_angles[0] = angleValue[0]*M_PI/180;    //Arm
                jointControl.joint_angles[1] = angleValue[1]*M_PI/180;       //Elbow 
                jointControl.joint_angles[2] = angleValue[2]*M_PI/180;    //Wrist
                jointControl.joint_angles[3] = angleValue[3]/100;       //Hand
                pubArm_.publish(jointControl);

                sleep(1);

                jointControl.joint_angles[1] = angleValue[5]*M_PI/180;       //Elbow straight
                pubArm_.publish(jointControl);
            break;
            case 's': //Arm down
                ROS_INFO("Arm going down");
                jointControl.joint_angles[0] = angleValue[4]*M_PI/180;
                jointControl.joint_angles[1] = angleValue[1]*M_PI/180;       //Elbow 
                jointControl.joint_angles[2] = angleValue[6]*M_PI/180;
                jointControl.joint_angles[3] = angleValue[7]/100;
                pubArm_.publish(jointControl);

                sleep(1);

                jointControl.joint_angles[1] = angleValue[5]*M_PI/180;       //Elbow straight
                pubArm_.publish(jointControl);
            break;
            default:
                std::cout <<"Unknown Command: "<<cmd<<" !\n";
            break;
        }
        loop_rate.sleep();
    }
    return 0;
}

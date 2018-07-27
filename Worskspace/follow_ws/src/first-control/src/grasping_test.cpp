/**
* \brief Pepper grasping an object on a table
* \author Vivien.C
* \date 18/07/2018
*
* Put pepper arm up or down
*
*/

#include <ros/ros.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <math.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    float angleValue[] = {
        25      //Arm table height
        ,60     //Elbow avoid up 
        ,100    //Hand open for grasping
        ,0      //Hand close
        ,100    //Arm down
        ,0.5    //Elbow straight
        ,119.5  //Arm back to avoid touching table
        ,75     //Hand not fully open
        ,10     //Elbow table height 
        ,0      //Wrist to basic
        ,-30    //Arm raises the object
    }; //Value for movement

    ros::init(argc, argv, "publisher_grasping");
    ros::NodeHandle n_;

    //Publisher
    ros::Publisher pubArm_ = n_.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 10);

    //Subscriber
    //pubSonarFront_ = n_.subscribe("/demo_control/sonar/front", 1, FrontDistance);
		
    ros::Rate loop_rate(1);

	//Initializaion of the movement
    naoqi_bridge_msgs::JointAnglesWithSpeed jointControl;
    jointControl.speed = 0.2;
    jointControl.joint_names.push_back("RShoulderPitch");
    jointControl.joint_names.push_back("RElbowRoll");
    jointControl.joint_names.push_back("RWristYaw"); 
    jointControl.joint_names.push_back("RHand");
    jointControl.joint_names.push_back("RShoulderRoll");
    jointControl.joint_angles.push_back(0);
    jointControl.joint_angles.push_back(0);
    jointControl.joint_angles.push_back(0);
    jointControl.joint_angles.push_back(0);
    jointControl.joint_angles.push_back(0);

    while (ros::ok())
    {
        std::cout << "\nUse z or s to start the movement:   ";
        char cmd[10]; //Just to make sure it's long enought
        std::cin.getline(cmd, 10);

        switch(cmd[0]){
            case 'z': //start movement
                ROS_INFO("Arm going up sequence");
                jointControl.joint_angles[0] = angleValue[6]*M_PI/180;  //Arm back
                jointControl.joint_angles[1] = angleValue[1]*M_PI/180;  //Elbow up
                jointControl.joint_angles[2] = angleValue[9]*M_PI/180;  //Wrist to normal
                pubArm_.publish(jointControl);

                sleep(1);
                jointControl.joint_angles[0] = angleValue[0]*M_PI/180;  //Arm to table height
                jointControl.joint_angles[3] = angleValue[2]/100;       //Hand open
                pubArm_.publish(jointControl);
                
                sleep(1);
                jointControl.joint_angles[1] = angleValue[8]*M_PI/180;  //Elbow straight
                pubArm_.publish(jointControl);

                sleep(2);
                ROS_INFO("Grasping");
                jointControl.joint_angles[3] = angleValue[3]/100;       //Hand close
                pubArm_.publish(jointControl);

                sleep(1);
                ROS_INFO("Testing");
                jointControl.joint_angles[0] = angleValue[10]*M_PI/180;      //Raises the object
                pubArm_.publish(jointControl);

                sleep(3);
                jointControl.joint_angles[0] = angleValue[0]*M_PI/180;
                pubArm_.publish(jointControl);

                sleep(1);
            break;


            case 's': //Return to base
                ROS_INFO("Arm going down");
                jointControl.joint_angles[0] = angleValue[4]*M_PI/180;  //Arm down
                jointControl.joint_angles[1] = angleValue[1]*M_PI/180;  //Elbow 
                jointControl.joint_angles[3] = angleValue[7]/100;
                pubArm_.publish(jointControl);

                sleep(1);
                jointControl.joint_angles[1] = angleValue[5]*M_PI/180;;       //Elbow straight
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

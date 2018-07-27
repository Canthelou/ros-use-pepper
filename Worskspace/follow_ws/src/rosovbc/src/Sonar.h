#include "sensor_msgs/PointCloud.h"
#include <geometry_msgs/Twist.h>
#define PI 3.14159265

#include <fstream>


using namespace std;

class Sonar
{

public:

    Sonar()
    {

        string sonarTopic, sonarMovements;


        m_nh.param("sonarTopic", sonarTopic, string(""));
        m_nh.param("sonarMovements", sonarMovements, string(""));
        m_nh.param("logs", m_logs_path, string(""));

        m_nh.getParam("sonarTopic", sonarTopic);
        m_nh.getParam("sonarMovements", sonarMovements);
        m_nh.getParam("logs", m_logs_path);

        stringstream str;
        str<<m_logs_path<<"logfile.txt";
        m_logfile.open(str.str().c_str());


        //
        // subscription to sonar topic
        //
        m_sonar_sub = m_nh.subscribe<sensor_msgs::PointCloud>(sonarTopic, 1, &Sonar::sonarCallback, this);

        //
        // advertise to RosAria
        //
        m_ros_pub = m_nh.advertise<geometry_msgs::Twist>(sonarMovements,1000);


    }

    void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud)
    {


//        m_velocity.linear.x = 0.1;
//        m_velocity.angular.z = 0.0;
//        m_ros_pub.publish(m_velocity);
//        ros::spinOnce();

        int sensors = pointCloud->points.size(); //set the number of sensors on the robot

        double r[16];
        double arg[16];
        double min_front = 100;
        double min_back = 100;
        double min_left =100;
        double min_right = 100;
        int index_front;
        int index_back;
        string turn_left_sensor;
        string turn_right_sensor;

        for(int i=1; i<7; i++)
        {
            r[i] = sqrt(pow(pointCloud->points[i].x,2) + pow(pointCloud->points[i].y,2));
//            arg[i] = atan(pointCloud->points[i].x / pointCloud->points[i].y)* 180 / PI;
            if (r[i] < min_front)
            {
                min_front = r[i];
                index_front=i;
            }
        }

        if ((min_front) < 0.50)
        {

            m_velocity.linear.x = 0.0;
            m_velocity.angular.z = 0.0;
            m_ros_pub.publish(m_velocity);
            ros::spinOnce();
        }
//        int angle_front;
//        if (index_front < 5){
//            angle_front = 270 + index_front * 22.5;
//        }
//        else if(index_front > 4){
//            angle_front = 22.5 + (index_front-4) * 22.5;
//        }

//        for(int i=9; i<sensors-1; i++)
//        {
//            r[i] = pow(pointCloud->points[i].x,2) + pow(pointCloud->points[i].y,2);
//            arg[i] = atan(pointCloud->points[i].x / pointCloud->points[i].y)* 180 / PI;
//            if (r[i] < min_back)
//            {
//                min_back = r[i];
//                index_back=i;
//            }
//        }

//        int angle_back;
//        if (index_back > 11)
//        {
//            angle_back = 180 + (-index_back + 15) * 22.5;
//        }
//        else if(index_back < 12)
//        {
//            angle_back = 22.5 + (index_back - 5) * 22.5 ;
//        }

//        double temp[2];
//        temp[0]=  pow(pointCloud->points[0].x,2) + pow(pointCloud->points[0].y,2);
//        temp[1]=  pow(pointCloud->points[8].x,2) + pow(pointCloud->points[8].y,2);

//        if (temp[0] > temp[1])
//        {
//            min_left = temp[1];
//            turn_left_sensor = "right";
//        }
//        else{
//            min_left = temp[0];
//            turn_left_sensor = "left";
//        }

//        temp[0]=  pow(pointCloud->points[7].x,2) + pow(pointCloud->points[7].y,2);
//        temp[1]=  pow(pointCloud->points[15].x,2) + pow(pointCloud->points[15].y,2);

//        if (temp[0] > temp[1])
//        {
//            min_right = temp[1];
//            turn_right_sensor = "left";
//        }

//        else
//        {
//            min_right = temp[0];
//            turn_right_sensor = "right";
//        }

//        sonar_values.distance_front = min_front;
//        sonar_values.angle_front = angle_front;
//        sonar_values.distance_back = min_back;
//        sonar_values.angle_back = angle_back;
//        sonar_values.turn_left = min_left;
//        sonar_values.turn_right = min_right;
//        sonar_values.turn_left_sensor = turn_left_sensor;
//        sonar_values.turn_right_sensor = turn_right_sensor ;
//        m_logfile<<"Sensors: "<<sensors<<endl;
        m_logfile<<"Front obstacle at: "<<min_front<< endl;//"  "<<angle_front<<endl;
//        m_logfile<<"Back obstacle at: "<<min_back<< "  "<<angle_back<<endl;
//        m_logfile<< "Left obstacle at: "<<min_left<<"  sensor: "<<turn_left_sensor<<endl;
//        m_logfile<< "Right obstacle at: "<<min_right<< "  sesnor: "<<turn_right_sensor<<endl;
    }

    ~Sonar()
    {

    }


private:
    ros::NodeHandle m_nh;

    ros::Publisher m_ros_pub;
    ros::Subscriber m_sonar_sub;

    ofstream m_logfile;
    string m_logs_path;

    geometry_msgs::Twist m_velocity;


};

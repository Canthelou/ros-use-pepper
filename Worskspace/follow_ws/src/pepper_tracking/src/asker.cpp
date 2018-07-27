/**
* \file asker.cpp
* \brief Do the bridge between the user ask and the image worker
* \author Vivien.C
* \date 02/05/2018
*
* The user input an image in a special format, and this node ask to find the area 
* of this image if it's possible.
*
*/


#include <ros/ros.h>		/* Ros library */
#include <sensor_msgs/Image.h>  /* Type of image_raw */
#include <pepper_tracking/image_request.h>	/* Msg for the request topic */
#include <pepper_tracking/image_return.h>	/* Msg for the return topic */
#include <time.h>       	/* time */
#include <string.h>       	/* string */
#include <vector>

		

/**
 * \class asker
 * \brief Allow the user to add/remove image to the list of goals of Pepper
 */
class asker
		
{
public:
		
	asker() //Constructor
	{
		//Publisher
		pubRequest_ = n_.advertise<pepper_tracking::image_request>("/pepper_tracking/image_request", 10);

		//Subscriber
		subReturn_ = n_.subscribe("/pepper_tracking/image_return", 1, &asker::imageReturn, this);
		
		//Prgm Var
	}


	/** Subcriber CallBack **/

	/**
	* \fn void imageReturn(const pepper_tracking::image_request &msg)
	* \brief Receive the callback of the worker concerning a goal success
	*
	* \param msg Custom msg from this package, pepper_tracking/image_return.msg
	* \return void
	*/
	void imageReturn(const pepper_tracking::image_return &msg){
		if(msg.found)
			removeImage(msg.image_name);
	}

	/** Other function **/

	/**
	 * \fn void addImage(sensor_msgs::Image newImage, std::string imageName)
	 * \brief Sending image searching request to the topic  
	 * 
	 * \param newImage The image on a sensor_msgs::Image format
	 * \param imageName Used as id for the new image
	 * \return void
	 */
	void addImage(sensor_msgs::Image image_data, std::string imageName){
		//Image research
		pepper_tracking::image_request request;

		request.task = "seek"; //Ordre de recherche
		request.image_name = imageName;
		request.image_data = image_data;
		
		pubRequest_.publish(request);
	}

	/**
	 * \fn void removeImage(std::string imageName)
	 * \brief When an image found is receive, publish the success
	 * 
	 * \param imageName Used as id for the image
	 * \return void 
	 */
	void removeImage(std::string imageName){
		//Image research done
		pepper_tracking::image_request request;

		request.task = "forget"; //Ordre d'oublie
		request.image_name = imageName;

		pubRequest_.publish(request);
	}

 	

private:
	//Subs/Pub
	ros::NodeHandle n_;
	ros::Publisher pubRequest_;
	ros::Subscriber subReturn_;

	//Timer
	ros::Timer walkingTimer;

	//Var
	
};//End of class asker


int main(int argc, char **argv)
{
	ROS_INFO("Ready to go !");
	ros::init(argc, argv, "subscribe_and_publish");

	asker myObject;

	ros::spin();

	return 0;
}


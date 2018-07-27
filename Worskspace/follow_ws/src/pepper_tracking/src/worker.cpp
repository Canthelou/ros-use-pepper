/**
* \file worker.cpp
* \brief Do the bridge between visp, pepper and the user node
* \author Vivien.C
* \date 02/05/2018
*
* Allow Pepper to find in the area an image ask by user
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
 * \class worker
 * \brief Allow Pepper to move and detect user wanted image 
 */
class worker
{
public:
	worker() //Constructor
	{
		//Publisher
		pubReturn_ = n_.advertise<pepper_tracking::image_return>("/pepper_tracking/image_return", 10);

		//Subscriber
		subRequest_ = n_.subscribe("/pepper_tracking/image_request", 1, &worker::imageRequest, this);
		subImage_raw_ = n_.subscribe("/camera/image_raw", 1, &worker::imageRaw, this);
		
		//Prgm Var
		sensor_msgs::Image ff;
		listImage[0] = ff;
	}


	/** Subcriber CallBack **/

	/**
	* \fn void imageRequest(const pepper_tracking::image_request &msg)
	* \brief Receive the new image request to find in the area
	*
	* \param msg Custom msg from this package, pepper_tracking/image_request.msg
	* \return void
	*/
	void imageRequest(const pepper_tracking::image_request &msg){
		if(msg.task == "seek"){
			//Adding the image
			listImage.push_back(msg.image_data);
			listNameImage.push_back(msg.image_name);
		} else{

			if(msg.task == "forget"){
				//Image Supression
				for(int i=0;i<listNameImage.size();i++){
					if(listNameImage[i] == msg.image_name){
						//Supp from both array
						listImage.erase(listImage.begin() + i);
						listNameImage.erase(listNameImage.begin() + i);
					}
				}
			}
		}
	}

	/**
	* \fn void imageRaw(const sensor_msgs::Image &msg)
	* \brief Allow to process the image_raw of Pepper
	*
	* \param msg Pepper camera output
	* \return void
	*/
	void imageRaw(const sensor_msgs::Image &msg){
		for(int i=0;listImage.size();i++)
			requestToVisp(listImage[i], msg);
	}

	/**
	 * \fn void requestToVisp(sensor_msgs::Image imageGoal, sensor_msgs::Image imageNao)
	 * \brief In charge of doing the request to compare both image to VIsp
	 * 
	 * 
	 * \param imageGoal All image that we want to find, one by one
	 * \param imageNao A frame of pepper vision
	 * \return void
	 */
	void requestToVisp(sensor_msgs::Image imageGoal, sensor_msgs::Image imageNao){
		
	}

	/**
	 * \fn void imageFound(std::string imageName)
	 * \brief When Visp say that two image are similar, publish the success
	 * 
	 * \param imageName Id/Name of the image found to inform the topic
	 * \return void 
	 */
	void imageFound(std::string imageName){
		//Publish the success
		pepper_tracking::image_return orderReturn;

		orderReturn.found = true;
		orderReturn.image_name = imageName; //Used as id

		pubReturn_.publish(orderReturn);
	}
 	

private:
	//Subs/Pub
	ros::NodeHandle n_;
	ros::Publisher pubReturn_;
	ros::Subscriber subRequest_, subImage_raw_;

	//Var list
	std::vector<sensor_msgs::Image> listImage;
	std::vector<std::string> listNameImage;

};//End of class worker


int main(int argc, char **argv)
{
	ROS_INFO("Ready to go !");
	ros::init(argc, argv, "subscribe_and_publish");

	worker myObject;

	ros::spin();

	return 0;
}


#include "DesiredImageManager.h"



DesiredImageManager::DesiredImageManager()
    : m_it(m_nh)
{

    string cameraTopic;
    m_nh.param("CameraTopic", cameraTopic, string(""));
    m_nh.param("logs", m_logs_path, string(""));
    m_nh.param("data", m_data_path,string(""));

    m_nh.getParam("logs", m_logs_path);
    m_nh.getParam("cameraTopic", cameraTopic);
    stringstream str;
    str<<m_logs_path<<"logfile.txt";
    m_logfile.open(str.str().c_str());

    m_width = m_visual_servo_tools.getWidth();
    m_height = m_visual_servo_tools.getHeight();

    m_iter = -1;
    m_desired_image.init(m_height,m_width);
    m_display_desired_image.init(m_desired_image, 0, 0, "desired_image");

    m_image_sub = m_it.subscribe(cameraTopic, 1, &DesiredImageManager::imageCallback, this);
}




DesiredImageManager::~DesiredImageManager()
{

}


void DesiredImageManager::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    m_logfile<<"############################### m_iter"<<m_iter<<endl;
    m_iter++;
    
    m_desired_image = visp_bridge::toVispImage(*image);
    if (m_iter  == 0)
    {
        std::string filename_write_image;
        stringstream ss_desired_image;
        ss_desired_image<<m_logs_path<<"desired_images/desired_image"<<".png";
        filename_write_image = vpIoTools::path(ss_desired_image.str().c_str());

        vpDisplay::display(m_desired_image);
        vpDisplay::flush(m_desired_image);
        vpImageIo::write(this->m_desired_image, filename_write_image);
        m_logfile<<"desired_image written to"<<filename_write_image<<endl;
    }
}

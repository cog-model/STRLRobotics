
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>  
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
class ImageConverter{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> syncro_;
    
    public:
    cv::Mat colorIm;
    cv::Mat depthIm;
    bool getDataFlag = false;
    std::string frame_id;
    ImageConverter(const std::string &imageTopic, const std::string &depthTopic);
    ~ImageConverter();
    

    protected:
    void imageCb(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::ImageConstPtr &depth);

};
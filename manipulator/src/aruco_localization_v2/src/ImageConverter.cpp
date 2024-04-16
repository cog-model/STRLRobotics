#include "ImageConverter.h"
ImageConverter::ImageConverter(const std::string &imageTopic, const std::string &depthTopic):
        it_(nh_),
        syncro_(image_sub_, depth_sub_, 60){

        using namespace sensor_msgs;
        image_sub_.subscribe(nh_, imageTopic, 1);
        depth_sub_.subscribe(nh_, depthTopic, 1);
        syncro_.registerCallback(boost::bind(&ImageConverter::imageCb, this, _1, _2));
    }

ImageConverter::~ImageConverter(){
        
    }

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::ImageConstPtr &depth){
        frame_id = img->header.frame_id;
        cv_bridge::CvImagePtr cv_ptrColor, cv_ptrDepth;
        try{
            cv_ptrColor = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            cv_ptrDepth = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("err: %s", e.what());
            return;
        }
        cv_ptrColor->image.copyTo(colorIm);
        cv_ptrDepth->image.copyTo( depthIm);
        getDataFlag = true;
    }
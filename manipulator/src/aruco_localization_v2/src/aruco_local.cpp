
#include "ImageConverter.h"
#include "ArucoLocalizer.h"
#include <thread>
std::string  NODENAME = "aruco_localizator";

int main(int argc, char** argv){
    std::string color_topic, depth_topic;
    ros::init(argc, argv, NODENAME);
    ros::NodeHandle nh("~");
    nh.param<std::string>("color_topic", color_topic, "/d400/color/image_raw");
    nh.param<std::string>("depth_topic", depth_topic, "/d400/aligned_depth_to_color/image_raw");

    ImageConverter ic(
        color_topic,
        depth_topic
    );
    auto inst = ArucoLocalizer::GetInstance();
    while (!ic.getDataFlag && !ros::isShuttingDown()){
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(16));
    }
    while (!ros::isShuttingDown()) {
        std::vector<std::vector<cv::Point3f>> buffer;
        std::vector<int> idsBuffer;
        inst->LocalizeMarkers(ic.colorIm, ic.depthIm, buffer, idsBuffer);
        //inst->DrawMarkersWithCords(ic.colorIm, buffer);
        inst->PublishPoints(idsBuffer, buffer, ic.frame_id);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(20));
    }
    
    return 0;
}
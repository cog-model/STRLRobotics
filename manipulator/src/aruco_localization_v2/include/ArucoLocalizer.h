
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#include "aruco_localization_v2/aruco_msg.h"
#include "aruco_localization_v2/aruco_array_msg.h"

class ArucoLocalizer{
    public:
    static ArucoLocalizer *GetInstance();
    void LocalizeMarkers(cv::Mat &colorIm, cv::Mat &depthIm, std::vector<std::vector<cv::Point3f>> &buffer, std::vector<int> &idsBuffer);
    void DrawMarkersWithCords(cv::Mat &colorIm, std::vector<std::vector<cv::Point3f>> cords);
    void PublishPoints(std::vector<int> &idsBuffer, std::vector<std::vector<cv::Point3f>> cords, std::string frame_id);
    protected:
    ros::NodeHandle nodeH;
    ros::Publisher arucoPublisher; 
    std::string topicName = "objects";
    float fx =  911.5072631835938, 
          fy =  909.490478515625,
          cx =  633.1768798828125, 
          cy =  373.49066162109375, 
          Tx =  0.0, 
          Ty =  0.0;
    void DetectMarkers(cv::Mat &image, std::vector<std::vector<cv::Point2f>> &resultBuffer, std::vector<int> &idsBuffer);
    cv::Point3f PointToGlobal(cv::Mat &color, cv::Mat &depth, cv::Point2f &point);
    static ArucoLocalizer *instance;
    ArucoLocalizer();
};

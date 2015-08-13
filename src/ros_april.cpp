#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"

#include <iostream>
#include <ctype.h>
#include <sstream>

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

ros::Publisher pose_pub;

double m_width(640);
double m_height(480);
double m_tagSize(0.1524); //A4
double m_fx(543); // for PS3
double m_fy(543); // for PS3
double m_px(m_width/2);
double m_py(m_height/2);

AprilTags::TagCodes m_tagCodes = AprilTags::tagCodes16h5;

AprilTags::TagDetector* m_tagDetector;

void camera_info_callback(sensor_msgs::CameraInfo camera_info){

	fprintf("called");
}

void print_detection(AprilTags::TagDetection& detection) {
  
  if(detection.id != 0)
    return;

  // recovering the relative pose of a tag:

  // NOTE: for this to be accurate, it is necessary to use the
  // actual camera parameters here as well as the actual tag size
  // (m_fx, m_fy, m_px, m_py, m_tagSize)

  Eigen::Vector3d cameraTranslation; //in camera frame
  Eigen::Matrix3d cameraRotation;    //in camera frame
  detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                           cameraTranslation, cameraRotation);
    
  //rotate into body frame
  Eigen::Matrix3d cToM; //camera orientation to body orientation
  cToM <<
    1, 0, 0,  //right of camera points to the right
    0,-1, 0, //Bottom of camera points backward
    0, 0,-1; //Front of camera faces down
  Eigen::Vector3d copterTranslation = cToM*cameraTranslation;

  //publish ros
  tf::Matrix3x3 mat;
  mat[0][0] = cToM(0,0); mat[0][1] = cToM(0,1); mat[0][2] = cToM(0,2);
  mat[1][0] = cToM(1,0); mat[1][1] = cToM(1,1); mat[1][2] = cToM(1,2);
  mat[2][0] = cToM(2,0); mat[2][1] = cToM(2,1); mat[2][2] = cToM(2,2);
  tf::Quaternion q;
  mat.getRotation(q);

  geometry_msgs::Pose to_publish;
  to_publish.position.x = copterTranslation(0);
  to_publish.position.y = copterTranslation(1);
  to_publish.position.z = copterTranslation(2);
  tf::quaternionTFToMsg(q, to_publish.orientation);

  pose_pub.publish(to_publish);
}

void image_callback(const sensor_msgs::ImageConstPtr& picture)
{

  cv_bridge::CvImageConstPtr bridge = cv_bridge::toCvShare(picture,"bgr8");
  cv_bridge::CvImage thresholdPublisher;
  sensor_msgs::Image rosImage;
  
  cv::Mat image_gray;
  
  cv::cvtColor(bridge->image, image_gray, CV_BGR2GRAY);
  vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
  return;
}


int main( int argc, char** argv )
{
  //ROS init
  ros::init(argc, argv, "apriltag_minimal");
  ros::NodeHandle n;

  // Grab camera info
  ros::Subscriber sub = n.subscribe("/MyWebCam/camera_info", 1, camera_info_callback);


  /*if(ros::names::remap("image") == "image")
  {
    ROS_INFO("image has not been mapped");
    return 0;
  }

  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

  pose_pub = n.advertise<geometry_msgs::Pose>("tag", 1);
  ros::Subscriber sub = n.subscribe(ros::names::remap("image"), 1, image_callback,ros::TransportHints().tcpNoDelay());*/
  
  ros::spin();

  return 0;
}


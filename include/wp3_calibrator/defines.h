#ifndef DEFINES_H
#define DEFINES_H

#include <string>

#define ROS_LOOPRATE 10
const double ARUCODIMENSION = 0.60; // Side length of Aruco markers



// Recording ---------------------
const int IMAGES_C = 400; // Number of color images to capture for aruco detection (Increase if detectionrate is low)
const int IMAGES_D = 50; // Number of depth images to capture for cloud and corner point calculation.

// Refinement (ICP) ------------- ACCURATE
//const int CLOUDS = 10; // Number of images to convert to clouds for refinement. Must be smaller than IMAGES_D (Requires time and memory)
//const float ICP_MAX_CORR_DIST = 1.0; // search radius for first round [m]
//const double ICP_CONVERGE = 1e-8;  // abs(curr_fitness - last_fitness) / curr_fitness
//const int ICP_MAX_ROUNDS = 10;  // second end criteria
//const int ICP_ITERATIONS = 100; // ICP iterations for each round (multiplied with round number)
//const double ICP_TRANS_EPSILON = 1e-14; // Transformation refinement resolution)

// Refinement (ICP) ------------- GOOD
const int CLOUDS = 5; // Number of images to convert to clouds for refinement. Must be smaller than IMAGES_D (Requires time and memory)
const float ICP_MAX_CORR_DIST = 1.0; // search radius for first round [m]
const double ICP_CONVERGE = 1e-6;  // abs(curr_fitness - last_fitness) / curr_fitness
const int ICP_MAX_ROUNDS = 10;  // second end criteria
const int ICP_ITERATIONS = 100; // ICP iterations for each round (multiplied with round number)
const double ICP_TRANS_EPSILON = 1e-12; // Transformation refinement resolution)

// Refinement (ICP) ------------- FAST
//const int CLOUDS = 2; // Number of images to convert to clouds for refinement. Must be smaller than IMAGES_D (Requires time and memory)
//const float ICP_MAX_CORR_DIST = 1.0; // search radius for first round [m]
//const double ICP_CONVERGE = 1e-6;  // abs(curr_fitness - last_fitness) / curr_fitness
//const int ICP_MAX_ROUNDS = 2;  // second end criteria
//const int ICP_ITERATIONS = 10; // ICP iterations for each round (multiplied with round number)
//const double ICP_TRANS_EPSILON = 1e-6; // Transformation refinement resolution)

// Visualization ------------------------
//#define VIEW_ICP // create a viewer to see ICP process.
#define VIEWERS_ENABLED
#define VIEW_IMAGES
#define SHOWDEPTH // view cropped depth images

enum Cropping {Rect, Mask};
const double PI = 3.14159265358979323846;  //PI

// added marker padding in %   (padding size/aruco size)
// eg. 0.5 squares around is 1.0/6.0 = 0.166666
const float PADDING = 2.0/6.0;


namespace wp3 {
//extern std::string package_path; // defined in arucoprocessor.cpp
const std::string package_path = "/home/sfi/catkin_ws/src/wp3_calibrator/";
} // end namespace wp3






//// STD
//#include <stdio.h>
//#include <stdlib.h>
//#include <iostream>
//#include <math.h>
//#include <fstream>
//#include <string>
//#include <unistd.h>
//#include <mutex>
//#include <thread>
//#include <vector>

//#include <curses.h> // keyboard input
// ROS
#include <ros/console.h>  // http://wiki.ros.org/roscpp/Overview/Logging
//#include <ros/ros.h>
//#include <ros/package.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
//#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PointStamped.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>

//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/aruco.hpp>
//#include "opencv2/opencv.hpp"
//#include <opencv2/imgproc/imgproc.hpp>
////#include "opencv2/opencv.hpp"

//#include <boost/thread/thread.hpp>

//// PCL
//#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/parse.h>
//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/filter.h>
////#include <pcl_ros/point_cloud.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/common/io.h>
//#include <pcl/io/pcd_io.h>
////#include <pcl/ros/conversions.h>
//// /usr/include/pcl-1.7/pcl/ros/conversions.h:44:2: warning: #warning The <pcl/ros/conversions.h> header is deprecated.
////please use <pcl/conversions.h> instead. [-Wcpp] #warning The <pcl/ros/conversions.h> header is deprecated. please use #include <pcl_conversions/pcl_conversions.h>


//// Registration
//#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/registration/ia_ransac.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/transformation_estimation_point_to_plane.h>

//// Filtering
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/random_sample.h>
//#include <pcl/filters/crop_box.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/bilateral.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/common/transforms.h>
//#include <pcl/filters/conditional_removal.h>

//// Libfreenect2
//#include <libfreenect2/registration.h>

// openCV for image processing
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/aruco.hpp>
#include "opencv2/opencv.hpp"
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/eigen.hpp>

typedef std::vector<cv::Point2f> Vec2fPoint;
typedef std::vector<cv::Point3f> Vec3fPoint;
typedef std::vector<std::vector<cv::Point2f>> VecVec2fPoint;
typedef std::vector<std::vector<cv::Point3f>> VecVec3fPoint;
#endif // DEFINES_H

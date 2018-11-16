#ifndef IMAGECONVERTER_H
#define IMAGECONVERTER_H

#include "wp3_calibrator/defines.h"

// STD
//#include <vector>
#include <mutex>

// openCV for image processing
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/aruco.hpp>
#include "opencv2/opencv.hpp"
//#include <opencv2/imgproc/imgproc.hpp>

// ROS
//#include <ros/ros.h>
//#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
////#include <tf/transform_Broadcaster.h>
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PointStamped.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>

// PCL
////#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

// Eigen for matrix calculations
//#include <Eigen/Core> // Matrix and Array classes, basic linear algebra (including triangular and selfadjoint products), array manipulation
//#include <Eigen/Geometry> // Transform, Translation, Scaling, Rotation2D and 3D rotations (Quaternion, AngleAxis)
//#include <Eigen/LU> // Inverse, determinant, LU decompositions with solver (FullPivLU, PartialPivLU)
//#include <Eigen/Cholesky> // LLT and LDLT Cholesky factorization with solver
//#include <Eigen/Householder> // Householder transformations; this module is used by several linear algebra modules
//#include <Eigen/SVD>  // SVD decompositions with least-squares solver (JacobiSVD, BDCSVD)
//#include <Eigen/QR> // QR decomposition with solver (HouseholderQR, ColPivHouseholderQR, FullPivHouseholderQR)
//#include <Eigen/Eigenvalues>  // Eigenvalue, eigenvector decompositions (EigenSolver, SelfAdjointEigenSolver, ComplexEigenSolver)
//#include <Eigen/Sparse> // Sparse matrix storage and related basic linear algebra (SparseMatrix, SparseVector)
//#include <Eigen/Dense>  // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header files
//#include <Eigen/Eigen>  // Includes Dense and Sparse header files (the whole Eigen library)

namespace wp3 {

class imageConverter
{
public:
  imageConverter(const std::string& inputName,
                 const std::string&  type,
                 image_transport::ImageTransport &image_transport_nh,
                 ros::NodeHandle &nodehandle);

  ~imageConverter();

  void getCurrentImage(cv::Mat *input_image);

  void getCurrentDepthMap(cv::Mat *input_image);

  void callback_depth(const sensor_msgs::ImageConstPtr& msg);

  void callback_color(const sensor_msgs::ImageConstPtr& msg);

private:
  //  std::string topicName_;
  cv::Mat src_image_, src_depth_;
  ros::NodeHandle nh_;
//  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  std::string topic_type_;
  ros::Time timestamp_;
  ros::Time last_frame;
  std::recursive_mutex i_mutex, d_mutex;
};



class depthProcessor
{
public:
  depthProcessor(const std::string& inputNameA,
                 ros::NodeHandle &nodehandle);

  ~depthProcessor();

  void get_filtered_PCL_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

  void callback_cloudA(const sensor_msgs::PointCloud2ConstPtr& input);

private:
  pcl::PointCloud<pcl::PointXYZ> tmp_cloud_read_;
  ros::Subscriber depth_subA_;
  ros::NodeHandle nh_;
  ros::Time timestamp_depth_;
  ros::Time last_frame_depth_;

};

} // end namespace wp3

#endif // IMAGECONVERTER_H

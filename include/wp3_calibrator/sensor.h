#ifndef SENSOR_H
#define SENSOR_H

#include "wp3_calibrator/imageconverter.h"
#include "wp3_calibrator/defines.h"

// STD
//#include <stdio.h>
//#include <stdlib.h>
//#include <iostream>
//#include <math.h>
//#include <fstream>
#include <string>
//#include <unistd.h>
//#include <mutex>
//#include <thread>
//#include <vector>

//#include <limits>

// openCV for image processing
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/aruco.hpp>
#include "opencv2/opencv.hpp"
//#include <opencv2/imgproc/imgproc.hpp>

// ROS
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>


// PCL
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/common/centroid.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/visualization/pcl_visualizer.h>

// Eigen for matrix calculations
//#include <Eigen/Core> // Matrix and Array classes, basic linear algebra
//(including triangular and selfadjoint products), array manipulation
//#include <Eigen/Geometry> // Transform, Translation, Scaling, Rotation2D and
//3D rotations (Quaternion, AngleAxis)
//#include <Eigen/LU> // Inverse, determinant, LU decompositions with solver
//(FullPivLU, PartialPivLU)
//#include <Eigen/Cholesky> // LLT and LDLT Cholesky factorization with solver
//#include <Eigen/Householder> // Householder transformations; this module is
//used by several linear algebra modules
//#include <Eigen/SVD>  // SVD decompositions with least-squares solver
//(JacobiSVD, BDCSVD)
//#include <Eigen/QR> // QR decomposition with solver (HouseholderQR,
//ColPivHouseholderQR, FullPivHouseholderQR)
//#include <Eigen/Eigenvalues>  // Eigenvalue, eigenvector decompositions
//(EigenSolver, SelfAdjointEigenSolver, ComplexEigenSolver)
//#include <Eigen/Sparse> // Sparse matrix storage and related basic linear
//algebra (SparseMatrix, SparseVector)
//#include <Eigen/Dense>  // Includes Core, Geometry, LU, Cholesky, SVD, QR, and
//Eigenvalues header files
//#include <Eigen/Eigen>  // Includes Dense and Sparse header files (the whole
//Eigen library)

namespace wp3
{
//namespace calibrate
//{

class Sensor
{
public:
  // Contructor
  Sensor(std::string name, ros::NodeHandle & nodehandle);

  // Destructor
  ~Sensor();

  // boost pointer
  typedef boost::shared_ptr<Sensor> Ptr;


  void clear();

  std::string name_;
//  void readTopics(std::string nodeA, std::string nodeB, cv::Mat* rgb_A,
//                  cv::Mat* depth_A, cv::Mat* rgb_B, cv::Mat* depth_B,
//                  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A,
//                  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal,
//                  bool update);
  void cameraInfoCallback (const sensor_msgs::CameraInfo::ConstPtr & msg);

  void readTopics(bool update);

  void appendClouds();

  // Accessors/getter/setter
  std::string getDepthTopic() const;
  void setDepthTopic(const std::string& value);

  std::string getImageTopic() const;
  void setImageTopic(const std::string& value);

  std::string getCloudTopic() const;
  void setCloudTopic(const std::string &cloudTopic);

  cv::Mat getImageMat();
  void setImageMat(const cv::Mat &imageMat);

  cv::Mat getDepthMat() const;
  void setDepthMat(const cv::Mat &depthMat);

  pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() const;
  void setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);



  //move to private?
  cv::Mat imageMat_;  // original color image
  cv::Mat depthMat_;  // original depth image
  std::vector<cv::Mat> imageMatVec_;  // original color images in a vector
  std::vector<cv::Mat> depthMatVec_;  // original depth image in a vector

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr_;      // original point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAccPtr_;   // original point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCrPtr_;    // original cropped point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1Ptr_;     // point cloud after 1 transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1AccPtr_;  // point cloud after 1 transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1CrPtr_;   // cropped point cloud after 1 transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2Ptr_;     // point cloud after 2 transformations
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2AccPtr_;   // point cloud after 2 transformations
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2CrPtr_;   // cropped point cloud after 2 transformations
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3Ptr_;     // point cloud after 3 transformations
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3AccPtr_;   // point cloud after 3 transformations
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3CrPtr_;   // cropped point cloud after 3 transformations


  void setCamera_info_topic(const std::string &camera_info_topic);

  Eigen::Matrix3d getIntrinsics_matrix() const;

  std::vector<double> getDistCoeffs() const;

  std::vector<cv::Mat> getImageMatVec() const;
  void clearImageMatVec();

  std::vector<cv::Mat> getDepthMatVec() const;

  std::string getTfTopic() const;
  void setTfTopic(const std::string &tfTopic);

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  std::string depthTopic_;
  std::string imageTopic_;
  std::string cloudTopic_;
  std::string camera_info_topic_;
  std::string tfTopic_;
  bool intrinsics_set_;
  Eigen::Matrix3d intrinsics_matrix_;
//  Eigen::MatrixXd distCoeffs_;
  std::vector<double> distCoeffs_;
//  sensor_msgs::CameraInfo::_D_type distCoeffs_;

//  cv::Mat imageMat_;
//  cv::Mat depthMat_;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr_;




}; // end class Sensor

//} // end namespace calibrate
} // end namespace wp3
#endif // SENSOR_H

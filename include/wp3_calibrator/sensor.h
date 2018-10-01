#ifndef SENSOR_H
#define SENSOR_H

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

class Sensor
{
public:
  // Contructor
  Sensor();

  // Destructor
  ~Sensor();

//  void readTopics(std::string nodeA, std::string nodeB, cv::Mat* rgb_A,
//                  cv::Mat* depth_A, cv::Mat* rgb_B, cv::Mat* depth_B,
//                  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A,
//                  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal,
//                  bool update);
  void readTopics(bool update);

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
  cv::Mat imageMat_;
  cv::Mat depthMat_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr_;



private:
  std::string depthTopic_;
  std::string imageTopic_;
  std::string cloudTopic_;

//  cv::Mat imageMat_;
//  cv::Mat depthMat_;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr_;




}; // end class Sensor

} // end namespace wp3
#endif // SENSOR_H

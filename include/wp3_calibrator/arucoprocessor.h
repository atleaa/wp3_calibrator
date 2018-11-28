#ifndef ARUCOPROCESSOR_H
#define ARUCOPROCESSOR_H

#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/functions.h"

// STD
#include <vector>

// openCV for image processing
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

// PCL
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen for matrix calculations
#include <Eigen/Core> // Matrix and Array classes, basic linear algebra (including triangular and selfadjoint products), array manipulation
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

typedef std::map<int, Eigen::Matrix4f> MarkerMapType;

namespace wp3 {



class arucoProcessor
{
public:
  // Constructor
  arucoProcessor();

  // Destrucor
  ~arucoProcessor();

  // boost pointer
  typedef boost::shared_ptr<arucoProcessor> Ptr;

  void clearAll();

  void processImages(Sensor &node,
                     MarkerMapType &transform4x4);
//  void detectMarkers(boost::ref(Sensor),
//                     MarkerMapType &transform4x4);

//  void makeCroppedCloud();

//  void getCroppedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  void createTransMatrix(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, Eigen::Matrix4f& tMat);

  void getAverageTransformation(Eigen::Matrix4f& transMat_avg, MarkerMapType& transMapUsed);

  pcl::PointCloud<pcl::PointXYZ>::Ptr getCroppedCloud() const;

  void viewImages(wp3::Sensor& node);

  void getMedianCornerPoints(VecVec2fPoint markerCorners,
                             std::vector<cv::Mat> &inputDepthVec,
                             Eigen::Matrix3d intrinsicMatrix,
                             VecVec3fPoint &markerCornerPoints);
  
//  void findBestPose(wp3::Sensor& node, std::vector<cv::Mat> inputDepthVec, Eigen::Matrix3d intrinsicMatrix);

private:
  void max4points(std::vector<cv::Point2f> cornerPoints, float & topx, float & topy, float & botx, float & boty, bool &flag);

  void pointcloudFromDepthImage (cv::Mat& depth_image,
                                 Eigen::Matrix3d &depth_intrinsics,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                                 bool crop,
                                 std::vector<int> c_ext,
                                 cv::Mat depth_aruco_dummy);

  // TODO, create vectors
//  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop1_; //(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop2_; //(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop3_; //(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud_; //(new pcl::PointCloud<pcl::PointXYZ>);
//  std::map<float, Eigen::Matrix4f> transformMap_;
  cv::Mat inputImage_;
  cv::Mat inputDepth_;
  cv::Mat distortionMat_;
  cv::Mat intrinsicMat_;
  MarkerMapType transformMap_;
  std::vector<cv::Vec3d> rotVecs_, transVecs_;
  std::vector<cv::Mat> maskVec_;
  std::vector<int> markerIdsMean_;
  VecVec2fPoint markerCornersMean_;
  int acc_; // iterator for accumulator
//  typedef std::pair<int, Eigen::Matrix4f>
//  cv::Mat current_depthMat_A_crop1_;
//  cv::Mat current_depthMat_A_crop2_;
//  cv::Mat current_depthMat_A_crop3_;
//  cv::Mat current_depthMat_B_crop1_;
//  cv::Mat current_depthMat_B_crop2_;
//  cv::Mat current_depthMat_B_crop3_;
  int findBestPose(std::vector<cv::Vec3d> &rotVecs,
                   std::vector<cv::Vec3d> &transVecs,
                   Vec3fPoint corners,
                   double &score);
};




} // end namespace wp3

#endif // ARUCOPROCESSOR_H

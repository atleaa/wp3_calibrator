#ifndef ARUCOPROCESSOR_H
#define ARUCOPROCESSOR_H

#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/functions.h"

// openCV for image processing
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>


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

namespace wp3 {



class arucoProcessor
{
public:
  // Contructor
  arucoProcessor();

  // Deconstrucor
  ~arucoProcessor();

  void detectMarkers(cv::Mat &inputImage,cv::Mat &inputDepth, Eigen::Matrix4f & transform4x4, std::string kinect_number);

  void getCroppedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  void createTransMatrix(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, Eigen::Matrix4f& tMat);

private:
  // TODO, create vectors
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop1_; //(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop2_; //(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop3_; //(new pcl::PointCloud<pcl::PointXYZ>);
  cv::Mat current_depthMat_A_crop1_;
  cv::Mat current_depthMat_A_crop2_;
  cv::Mat current_depthMat_A_crop3_;
  cv::Mat current_depthMat_B_crop1_;
  cv::Mat current_depthMat_B_crop2_;
  cv::Mat current_depthMat_B_crop3_;
};

} // end namespace wp3

#endif // ARUCOPROCESSOR_H

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

//#include <iostream>
//#include <fstream>
//#include <string.h>
//#include <Eigen/Eigen>
//#include <limits>
//#include <pcl/filters/extract_indices.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>

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
//#include <opencv2/highgui/highgui.hpp>

// STD
#include <vector>

// openCV for image processing
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/aruco.hpp>
#include "opencv2/opencv.hpp"
//#include <opencv2/imgproc/imgproc.hpp>

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



namespace wp3
{


void max4points(std::vector<cv::Point2f> cornerPoints, float & topx, float & topy, float & botx, float & boty, bool &flag);

void pointcloudFromDepthImage (cv::Mat& depth_image,
                               Eigen::Matrix3f& depth_intrinsics,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                               bool crop,
                               std::vector<float> c_ext,
                               cv::Mat depth_aruco_dummy);
}


#endif // FUNCTIONS_H

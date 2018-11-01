#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/sensor.h"
#include "wp3_calibrator/visualization.h"

// STD
//#include <stdio.h>
//#include <stdlib.h>
#include <iostream>
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/visualization/pcl_visualizer.h>

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
#include <Eigen/Dense>  // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header files
//#include <Eigen/Eigen>  // Includes Dense and Sparse header files (the whole Eigen library)


// Averaging_quaternions
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <tf/transform_datatypes.h>


namespace wp3
{
void init_reference(std::string kinect_number);

void openGlobalReference(Eigen::Matrix4f & transf_to_open, std::string kinect_number);

////void readTopics(std::string nodeA, std::string nodeB, cv::Mat* rgb_A, cv::Mat* depth_A, cv::Mat* rgb_B, cv::Mat* depth_B, bool update);
//void readTopics(std::string nodeA,
//                std::string nodeB,
//                cv::Mat* rgb_A,
//                cv::Mat* depth_A,
//                cv::Mat* rgb_B,
//                cv::Mat* depth_B,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB_cropTotal,
//                bool update);

void ICP_allign(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_xyz_org,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_xyz_org,
                Eigen::Affine3f & transform_ICP,
                bool & converge_flag, float distThresh, int iter, double & fitnessScore);

void cloudPassthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered,
                            float* filter_limits);

bool isRotationMatrix(cv::Mat &R);

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);

void saveResults(Eigen::Matrix4f transf_to_save, double ICP2_fitnessScore, std::string kinect_number);

void readGlobalPose(std::string kinect_number, Eigen::Matrix4f & tMat);




#ifdef VIEW_ICP
void calcTransMats(wp3::Sensor &sensorA, wp3::Sensor &sensorB,
                   Eigen::Matrix4f transform_A, Eigen::Matrix4f transform_B,
                   Eigen::Matrix4f transform_reference_global, Eigen::Matrix4f & world_to_B, double & fitnessScore,
//                   pcl::visualization::PCLVisualizer viewerICP);
                   Visualization &viewer);
#else
void calcTransMats(wp3::Sensor &sensorA, wp3::Sensor &sensorB,
                   Eigen::Matrix4f transform_A, Eigen::Matrix4f transform_B,
                   Eigen::Matrix4f transform_reference_global, Eigen::Matrix4f & world_to_B, double & fitnessScore_to_print);
#endif

Eigen::Quaternionf getAverageQuaternion(const std::vector<Eigen::Quaternionf> &quaternions,
                                        const std::vector<float> &weights);

} // end namespace wp3


#endif // FUNCTIONS_H

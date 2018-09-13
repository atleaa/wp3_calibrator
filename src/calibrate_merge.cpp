// STD
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <vector>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
//#include <tf/transform_Broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// Registration
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

// Filtering
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>

// Libfreenect2
#include <libfreenect2/registration.h>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclVisColorCustom;
typedef pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> pclVisColorGeneric;

cv::Mat current_image_A;
cv::Mat current_depthMat_A;
cv::Mat current_depthMat_A_crop1;
cv::Mat current_depthMat_A_crop2;
cv::Mat current_depthMat_A_crop3;

cv::Mat current_image_B;
cv::Mat current_depthMat_B;
cv::Mat current_depthMat_B_crop1;
cv::Mat current_depthMat_B_crop2;
cv::Mat current_depthMat_B_crop3;


cv::Mat rot_mat(3, 3, cv::DataType<float>::type);

ros::Time timestamp;
ros::Time last_frame;
ros::Time timestamp_depth;
ros::Time last_frame_depth;

std::recursive_mutex r_mutex, r_mutex2;
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B_cropped(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B_cropped_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud2_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud3_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud4(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud5(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud5_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud6(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud6_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP1_AtoB (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP2_aTob (new pcl::PointCloud<pcl::PointXYZ>);

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

pcl::visualization::PCLVisualizer viewer1 ("Visualizer");
pcl::visualization::PCLVisualizer viewer2 ("Visualizer2");

Eigen::Matrix4f transform_A = Eigen::Matrix4f::Identity();
Eigen::Matrix4f transform_B = Eigen::Matrix4f::Identity();
Eigen::Affine3f transform_ICP = Eigen::Affine3f::Identity();
Eigen::Affine3f transform_ICP2 = Eigen::Affine3f::Identity();
Eigen::Matrix4f transform_ATOb;
Eigen::Matrix4f world_to_reference = Eigen::Matrix4f::Identity();
Eigen::Matrix4f world_to_B;
Eigen::Matrix4f transform_ICP1_print = Eigen::Matrix4f::Identity();
Eigen::Matrix4f transform_reference_global = Eigen::Matrix4f::Identity();

double ICP1_fitness_to_print;

double ICP2_fitness_1;
double ICP2_fitness_2;
double ICP2_fitness_3;
double ICP2_fitness_5;
double ICP2_fitness_6;
std::vector<double> ICP2_scores;
std::vector<Eigen::Matrix4f> transformations_ICP1_global;

std::string package_path = "/home/sfi/catkin_ws/src/WP3_ROS_Master/aruco_registration/";


void ICP_allign(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_xyz_org, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_xyz_org, Eigen::Affine3f & transform_ICP, bool & converge_flag, float distThresh, double & fitnessScore)
{
	// >>>>>>>> Registration by ICP <<<<<<<<<
	// Declare output point cloud and initialize ICP object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_robot_ICP(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	std::vector<int> indices1;
	std::vector<int> indices2;
	pcl::removeNaNFromPointCloud(*cloud_source_xyz_org,*cloud_source_xyz_org, indices1);
	pcl::removeNaNFromPointCloud(*cloud_target_xyz_org,*cloud_target_xyz_org, indices2);


	// Declare ICP parameters
	icp.setInputSource(cloud_source_xyz_org);
	icp.setInputTarget(cloud_target_xyz_org);
	icp.setMaxCorrespondenceDistance (distThresh);
	icp.setTransformationEpsilon(0.001);
	icp.setMaximumIterations (1000);
	// Perform the aligment
	icp.align(*cloud_robot_ICP);

	// Check for convergence
	fitnessScore = 1000; // if not converged
	converge_flag = icp.hasConverged();
	if (converge_flag)
	{
		transform_ICP = icp.getFinalTransformation();
		fitnessScore = icp.getFitnessScore();
		std::cout << "ICP converged with fitness score: " <<  icp.getFitnessScore() << std::endl;
	}
	else
	{
		std::cout << "ICP did not converge!" << std::endl;
	}

	cloud_robot_ICP->empty();
}




class depthProcessor
{
	pcl::PointCloud<pcl::PointXYZ> tmp_cloud_read;
	ros::Subscriber depth_subA;
	ros::NodeHandle nh_depthA;

public:
	depthProcessor(const std::string& inputNameA)
	{
		depth_subA = nh_depthA.subscribe(inputNameA, 1, &depthProcessor::callback_cloudA, this);// this refers to the non static object  (here: a depthProcessor object used in main)			depth_subB = nh_depthB.subscribe(inputNameB, 1, &depthProcessor::callback_cloudB, this);// this refers to the non static object  (here: a depthProcessor object used in main)
	}

	~depthProcessor()
	{
		depth_subA.shutdown();
		std::cout << "ROS stopped Depth Import" << std::endl;
	}

	void get_filtered_PCL_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
	{
	    while((timestamp_depth.toSec() - last_frame_depth.toSec()) <= 0) {
	        usleep(2000);
	        ros::spinOnce();
	    }

	    pcl::copyPointCloud(tmp_cloud_read, *cloud_ptr);
//	    *cloud_ptr = tmp_cloud_read; // also a possibility
	    last_frame_depth = timestamp_depth;
	}

	void callback_cloudA(const sensor_msgs::PointCloud2ConstPtr& input)
	{

		ros::Time frame_time_depth = ros::Time::now();
		timestamp_depth = frame_time_depth;
		pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);

		pcl_conversions::toPCL(*input, *cloud_filtered);
		pcl::fromPCLPointCloud2(*cloud_filtered, tmp_cloud_read);
	}
};

void cloudPassthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered, float* filter_limits)
{
	pcl::PassThrough<pcl::PointXYZ> passthrough; // note passthrough works only one filter at a time
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
	passthrough.setInputCloud (cloud);
	passthrough.setFilterFieldName ("x");
	passthrough.setFilterLimits (filter_limits[0], filter_limits[1]);
	passthrough.filter (*cloud_filtered);

	passthrough.setInputCloud (cloud_filtered);
	passthrough.setFilterFieldName ("y");
	passthrough.setFilterLimits (filter_limits[2], filter_limits[3]);
	passthrough.filter (*cloud_filtered);

	passthrough.setInputCloud (cloud_filtered);
	passthrough.setFilterFieldName ("z");
	passthrough.setFilterLimits (filter_limits[4], filter_limits[5]);
	passthrough.filter (*cloud_filtered);
}





// Visualizer is not meant for multithreaded applications!!
template <typename T_color> void visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, std::string name,
			pcl::visualization::PCLVisualizer * viewer, T_color * color, int point_size)
{
	viewer->removePointCloud(name);
	viewer->addPointCloud(cloud, *color, name);
//	Eigen::Affine3f tt;
//	tt = Eigen::Translation3f(0.,0.,0.);
//	viewer->addCoordinateSystem(1, tt, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
	viewer->spinOnce();
}



// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    std::cout << "should be identity: " << shouldBeIdentity << std::endl;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{


//    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);



}

void readTopics(const char* nodeA, const char* nodeB, bool update = false)
{
	// Create char arrays
	char point_cloud_A[64] = "/master/jetson";
	char point_cloud_B[64] = "/master/jetson";

	strcat(point_cloud_A, nodeA);
	strcat(point_cloud_A, "/points");
	strcat(point_cloud_B, nodeB);
	strcat(point_cloud_B, "/points");

	std::cout << "pointA_name: " << point_cloud_A << std::endl;
	std::cout << "pointB_name: " << point_cloud_B << std::endl;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	depthProcessor dp_A = depthProcessor(point_cloud_A); // to fix: src_cloud in this class is single-linked in callback, quick fix-> create two src_clouds
	depthProcessor dp_B = depthProcessor(point_cloud_B); // to fix: src_cloud in this class is single-linked in callback, quick fix-> create two src_clouds

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	//// World data from Atle
	//
	//	//0.185 7.360 5.07 -1.590 0 -2.59
	//	Eigen::Matrix4f world_to_reference = Eigen::Matrix4f::Identity();
	//	Eigen::Matrix4f world_to_B;
	//	Eigen::Matrix4f world_to_B_inverse;
	//	Eigen::Affine3f tf4cb = Eigen::Affine3f::Identity();
	//	tf4cb.translation() << 0.138, 7.437, 4.930;
	//	tf4cb.rotate (Eigen::AngleAxisf (-1.565, Eigen::Vector3f::UnitZ()));
	//	tf4cb.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitY()));
	//	tf4cb.rotate (Eigen::AngleAxisf (-2.590, Eigen::Vector3f::UnitX()));
	//	world_to_reference = tf4cb.matrix();
	//	kinect6_to_4 =  world_to_reference.inverse()* global_pose_kinect6;
	//	kinect3_to_4 =  world_to_reference.inverse()* global_pose_kinect3;
	//	kinect5_to_4 =  world_to_reference.inverse()* global_pose_kinect5;
	//	kinect2_to_4 =  world_to_reference.inverse()* global_pose_kinect2;
	//
	//
	//// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Data acquisition ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//	bool reading_complete_a;
		if (update)
		{
			current_cloud_A->clear();
			current_cloud_B->clear();
			src_cloudA_cropTotal->clear();
			src_cloudB_cropTotal->clear();

			std::cout << "emptied cloud, size now: " << current_cloud_A->size() << std::endl;
		}

		std::cout << "Reading cloud A... ";
		while( current_cloud_A->size() == 0)
		{
			dp_A.get_filtered_PCL_cloud(current_cloud_A);
		}
		std::cout << "done" << std::endl;
		std::cout << "Reading cloud B... ";

		while( current_cloud_B->size() == 0)
		{
			dp_B.get_filtered_PCL_cloud(current_cloud_B);
		}
		std::cout << "done" << std::endl;

		std::cout << "Reading point clouds from A and B 5 times... ";
}

void visualizeRegisSteps(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudA_init, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudB_init,
						 pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudA_Aruco, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudA_ROI,
						 pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudA_ICP)
{

}
void calcTransMats(Eigen::Matrix4f transform_A, Eigen::Matrix4f transform_B, Eigen::Matrix4f & transform_ICP1, double & fitnessScore_to_print)
{

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to camera B (Aruco) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	transform_ATOb = transform_B*transform_A.inverse();
	pcl::transformPointCloud (*src_cloudA_cropTotal, *cloudA_to_B_cropped, transform_ATOb);
	pcl::transformPointCloud (*current_cloud_A, *cloudA_to_B, transform_ATOb);
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to camera B (ROI ICP) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	bool ICP1_converged;
	double fitnessScore1;
	ICP_allign(cloudA_to_B_cropped,src_cloudB_cropTotal,transform_ICP, ICP1_converged, 0.2, fitnessScore1);

	if (ICP1_converged)
	{
		Eigen::Matrix4f transform_AtoB_ICP = transform_ICP.matrix()*transform_B*transform_A.inverse();
		std::cout << "Transformation ICP1: "<< transform_AtoB_ICP << std::endl;
		pcl::transformPointCloud (*src_cloudA_cropTotal, *cloudICP, transform_AtoB_ICP);
		pcl::transformPointCloud (*current_cloud_A, *cloudICP1_AtoB, transform_AtoB_ICP);
		transform_ICP1 = transform_AtoB_ICP; // value to be written
	}
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to Camera B (Final Fine ICP) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	bool ICP2_converged;
	double fitnessScore2;
	ICP_allign(cloudICP1_AtoB,current_cloud_B,transform_ICP2, ICP2_converged,0.3, fitnessScore2);
	fitnessScore_to_print = fitnessScore2;
	Eigen::Matrix4f transform_AtoB_ICP2;
	if (ICP2_converged)
	{
		transform_AtoB_ICP2 = transform_ICP2.matrix()*transform_ICP.matrix()*transform_B*transform_A.inverse();
		std::cout << "Transformation ICP2: "<< transform_AtoB_ICP2 << std::endl;
		pcl::transformPointCloud (*current_cloud_A, *cloudICP2_aTob, transform_AtoB_ICP2);

	}
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	world_to_B = world_to_reference*transform_AtoB_ICP2.inverse();
}
void saveResults(Eigen::Matrix4f transf_to_open, double ICP2_fitnessScore, const char* kinect_number)
{
	// The thought here is to save the final fine ICP results and the clouds before this transformation. Based on
	// the algorithm presented in the paper, the cloud with the lowest (best) ICP score is merged with the reference cloud.
	// New ICP scores are then calculated, and the selection process continues until all clouds have been merged.

	// Convert to openCV matrix first
	cv::Mat transf_to_open_openCV = cv::Mat::eye(4, 4, CV_64F);
	transf_to_open_openCV.at<double>(0,0) = transf_to_open(0,0);
	transf_to_open_openCV.at<double>(1,0) = transf_to_open(1,0);
	transf_to_open_openCV.at<double>(2,0) = transf_to_open(2,0);
	transf_to_open_openCV.at<double>(0,1) = transf_to_open(0,1);
	transf_to_open_openCV.at<double>(1,1) = transf_to_open(1,1);
	transf_to_open_openCV.at<double>(2,1) = transf_to_open(2,1);
	transf_to_open_openCV.at<double>(0,2) = transf_to_open(0,2);
	transf_to_open_openCV.at<double>(1,2) = transf_to_open(1,2);
	transf_to_open_openCV.at<double>(2,2) = transf_to_open(2,2);
	transf_to_open_openCV.at<double>(0,3) = transf_to_open(0,3);
	transf_to_open_openCV.at<double>(1,3) = transf_to_open(1,3);
	transf_to_open_openCV.at<double>(2,3) = transf_to_open(2,3);

	char fs_filename[64] = "src/aruco_registration/kinect_ICP1_tMat/kinect";
	strcat(fs_filename,kinect_number);
	strcat(fs_filename,"/results.yaml");
	std::cout << "Stored results in: " << fs_filename << std::endl;
	cv::FileStorage fs_result(fs_filename, cv::FileStorage::WRITE);
	fs_result << "ICP2FitnessScore" << ICP2_fitnessScore;
	fs_result << "ICP1_transformation" << transf_to_open_openCV;
	fs_result.release();

}
void openResults(Eigen::Matrix4f & transf_to_open, double & ICP2_fitnessScore, const char* kinect_number)
{
	// The thought here is to open the final fine ICP results and the transformation from the first ICP. Based on
	// the algorithm presented in the paper, the cloud with the lowest (best) ICP score is merged with the reference cloud.
	// New ICP scores are then calculated, and the selection process continues until all clouds have been merged.

	cv::Mat transf_to_open_openCV = cv::Mat::eye(4, 4, CV_64F);
        std::string fs_filename = package_path + "kinect_ICP1_tMat/kinect" + kinect_number + "/results.yaml";
	std::cout << "Opened results from: " << fs_filename << std::endl;
	cv::FileStorage fs_result(fs_filename, cv::FileStorage::READ);



	if (ICP2_fitnessScore == -1000)
	{
		fs_result["Global_transformation"] >> transf_to_open_openCV;

	}
	else
	{
		fs_result["ICP2FitnessScore"] >> ICP2_fitnessScore;
		ICP2_scores.push_back(ICP2_fitnessScore);
		ICP2_scores.push_back(2212);
		fs_result["Global_transformation_ICP1"] >> transf_to_open_openCV;
	}
	fs_result.release();
	// Convert to eigen matrix

	transf_to_open(0,0) = transf_to_open_openCV.at<double>(0,0);
	transf_to_open(1,0) = transf_to_open_openCV.at<double>(1,0);
	transf_to_open(2,0) = transf_to_open_openCV.at<double>(2,0);
	transf_to_open(0,1) = transf_to_open_openCV.at<double>(0,1);
	transf_to_open(1,1) = transf_to_open_openCV.at<double>(1,1);
	transf_to_open(2,1) = transf_to_open_openCV.at<double>(2,1);
	transf_to_open(0,2) = transf_to_open_openCV.at<double>(0,2);
	transf_to_open(1,2) = transf_to_open_openCV.at<double>(1,2);
	transf_to_open(2,2) = transf_to_open_openCV.at<double>(2,2);
	transf_to_open(0,3) = transf_to_open_openCV.at<double>(0,3);
	transf_to_open(1,3) = transf_to_open_openCV.at<double>(1,3);
	transf_to_open(2,3) = transf_to_open_openCV.at<double>(2,3);
}



int main (int argc, char** argv)
{
	// ROS messaging init
	std::cout << "Starting: "<<  std::endl;
	ros::init(argc, argv, "aruco_tf_publisher");
	ros::NodeHandle nh;
	ros::spinOnce();

	double dummy = -1000;
        const char* reference_node = {"1"};
        const char* calibration_order_initial[] = {"2", "4", "5", "3", "6"};
	std::vector<int> calibration_order_idx = {0, 1, 2, 3, 4};
//	calibration_order_idx.erase(calibration_order_idx.begin()+1);
	std::cout << calibration_order_idx[1] << std::endl;
	size_t numel_calib = sizeof(calibration_order_initial)/sizeof(calibration_order_initial[0]);
//	openResults(transform_reference_global, dummy, reference_node);
//	delete calibration_order_initial[1];
	std::cout << calibration_order_initial[1] << std::endl;
        openResults(transform_ICP1_print, ICP1_fitness_to_print, "2");
	std::cout << ICP2_scores[0] << std::endl;
//	for (size_t i = 0; i < numel_calib; i++)openResults(transform_ICP1_print, ICP1_fitness_to_print, "6");
//	{
//		openResults(transform_ICP1_print, ICP1_fitness_to_print, "6");
//	}
//	std::cout << ICP1_fitness_to_print << std::endl;
//	std::cout << transform_ICP1_print << std::endl;
//
//	readTopics(reference_node,calibration_order_initial[0],true);




}

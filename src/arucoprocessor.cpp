#include "wp3_calibrator/arucoprocessor.h"



namespace wp3 {

//std::string package_path = "/home/sfi/catkin_ws/src/wp3_calibrator/";

// Constructor
arucoProcessor::arucoProcessor() :
  src_cloud_crop1_(new pcl::PointCloud<pcl::PointXYZ>),
  src_cloud_crop2_(new pcl::PointCloud<pcl::PointXYZ>),
  src_cloud_crop3_(new pcl::PointCloud<pcl::PointXYZ>)
{

}


// Destrucor
arucoProcessor::~arucoProcessor()
{
 // boost::shared_ptr are autmatically removed when leaving scope, but can be cleared using ptr.reset();
}


void arucoProcessor::max4points(std::vector<cv::Point2f> cornerPoints, float & topx, float & topy, float & botx, float & boty, bool &flag)
{
  //	std::shared_ptr<Point2f> john;
  std::vector<cv::Point2f> temp_max_values = cornerPoints;
  std::vector<cv::Point2f> temp_min_values = cornerPoints;
  float max_factor_x = 5;
  float min_factor_x = 5;
  float max_factor_y = 5;
  float min_factor_y = 5;
  flag = false;
  // Turn the flag "true" for invalid points

  for (int z = 0; z < 4; z++)
  {
    if ((cornerPoints[z].x < 0.1 && cornerPoints[z].x > 3000 ) || (cornerPoints[z].y < 0.1 && cornerPoints[z].y > 3000 ))
    {
      flag = true;
    }
    if (flag)
    {
      break;
    }
  }

  if (!flag)
  {
    topx = std::max(std::max(max_factor_x+temp_max_values[0].x,max_factor_x+temp_max_values[1].x),std::max(max_factor_x+temp_max_values[2].x,max_factor_x+temp_max_values[3].x));
    topx = floor(topx);

    topy = std::max(std::max(max_factor_y+temp_max_values[0].y,max_factor_y+temp_max_values[1].y),std::max(max_factor_y+temp_max_values[2].y,max_factor_y+temp_max_values[3].y));
    topy = floor(topy);

    botx = std::min(std::min(-min_factor_x+temp_min_values[0].x,-min_factor_x+temp_min_values[1].x),std::min(-min_factor_x+temp_min_values[2].x,-min_factor_x+temp_min_values[3].x));
    botx = floor(botx);

    boty = std::min(std::min(-min_factor_y+temp_min_values[0].y,-min_factor_y+temp_min_values[1].y),std::min(-min_factor_y+temp_min_values[2].y,-min_factor_y+temp_min_values[3].y));
    boty = floor(boty);
  }

  // Check if the padding is out of bound
  if ((topx > 1920) || (topy > 1080) || (topx <= 0) || (topy <= 0))
  {
    flag = true;
  }

  // Deallocate memory
  std::vector<cv::Point2f>().swap(temp_min_values);
  std::vector<cv::Point2f>().swap(temp_max_values);
}


void arucoProcessor::pointcloudFromDepthImage (cv::Mat& depth_image,
                               Eigen::Matrix3f& depth_intrinsics,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                               bool crop = false,
                               std::vector<float> c_ext = {},
                               cv::Mat depth_aruco_dummy = {})
{
  // For point clouds XYZ
  float depth_focal_inverted_x = 1/depth_intrinsics(0,0);  // 1/fx
  float depth_focal_inverted_y = 1/depth_intrinsics(1,1);  // 1/fy

  pcl::PointXYZ new_point;


  if (!crop)
  {
    output_cloud->points.resize(depth_image.cols*depth_image.rows, new_point);
    output_cloud->width = depth_image.cols;
    output_cloud->height = depth_image.rows;
    output_cloud->is_dense = false;

    for (int i=0;i<depth_image.rows;i++)
    {
      for (int j=0;j<depth_image.cols;j++)
      {
        float depth_value = depth_image.at<float>(i,j);

        if (depth_value > 0)
        {
          // Find 3D position with respect to depth frame:
          new_point.z = depth_value;
          new_point.x = (j - depth_intrinsics(0,2)) * new_point.z * depth_focal_inverted_x;
          new_point.y = (i - depth_intrinsics(1,2)) * new_point.z * depth_focal_inverted_y;
          output_cloud->at(j,i) = new_point;
        }

        else
        {
          new_point.z = std::numeric_limits<float>::quiet_NaN();
          new_point.x = std::numeric_limits<float>::quiet_NaN();
          new_point.y = std::numeric_limits<float>::quiet_NaN();
          output_cloud->at(j,i) = new_point;
        }
      }
    }
  }
  else
  {
    //	  aruco_cornerpoints = {botx, boty, topx, topy};
    output_cloud->points.resize((depth_aruco_dummy.cols)*(depth_aruco_dummy.rows), new_point);
    output_cloud->width = depth_aruco_dummy.cols;
    output_cloud->height = depth_aruco_dummy.rows;
    output_cloud->is_dense = false;
    float bot_row = c_ext[0];
    float top_row = c_ext[2];
    float bot_col = c_ext[1];
    float top_col = c_ext[3];

    for (int k=bot_col;k<top_col;k++)
    {
      for (int m=bot_row;m<top_row;m++)
      {
        float depth_value = depth_image.at<float>(k,m);

        if (depth_value > 0)
        {
          // Find 3D position with respect to depth frame:
          new_point.z = depth_value;
          new_point.x = (m - depth_intrinsics(0,2)) * new_point.z * depth_focal_inverted_x;
          new_point.y = (k - depth_intrinsics(1,2)) * new_point.z * depth_focal_inverted_y;
          output_cloud->at(m-bot_row, k-bot_col) = new_point;
        }

        else
        {
          new_point.z = std::numeric_limits<float>::quiet_NaN();
          new_point.x = std::numeric_limits<float>::quiet_NaN();
          new_point.y = std::numeric_limits<float>::quiet_NaN();
          output_cloud->at(m-bot_row, k-bot_col) = new_point;
        }
      }
    }
  }
}


//void arucoProcessor::detectMarkers(cv::Mat &inputImage,cv::Mat &inputDepth, Eigen::Matrix4f & transform4x4, std::string kinect_number)
void arucoProcessor::detectMarkers(cv::Mat & inputImage, cv::Mat & inputDepth,
                                   std::map<int, Eigen::Matrix4f> &transform4x4,
                                   std::string kinect_number)
{

  std::string camera_name = "/kinect" + kinect_number;
  std::string filename_inputPicture = "Input Picture" + camera_name;
  std::string filename_crop1 = "cropped image 1" + camera_name;
  std::string filename_crop2 = "cropped image 2" + camera_name;
  std::string rgb_intrinsics_dir = package_path + "kinect_calibrations" + camera_name + "/calib_color.yaml";
  std::string filename_crop3 = "cropped image 3" + camera_name;

  std::cout << rgb_intrinsics_dir << std::endl;

  // TULL, use custom parameters!?
  // Default RGB parameters -> will be used in rectified depthMatrix since it is mapped onto RGB
  // Try to upscale IR intrinsics to RGB size--> use them to transform depth map to point cloud
  float const fx_default = 1081.37;
  float const fy_default = 1081.37;
  float const cx_default = 959.5;
  float const cy_default = 539.5;

  Eigen::Matrix3f ir_params;
  ir_params(0,0) = fx_default;
  ir_params(1,1) = fy_default;
  ir_params(0,2) = cx_default;
  ir_params(1,2) = cy_default;

  src_cloud_crop1_->clear();
  src_cloud_crop2_->clear();
  src_cloud_crop3_->clear();

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||

  float topx, topy, botx, boty;
  bool invalid_points = true;

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Cam. coeff. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  cv::Mat cameraMatrix;
  cv::Mat dist_coeffs;

  cv::FileStorage fs_calibration(rgb_intrinsics_dir, cv::FileStorage::READ);
  fs_calibration["cameraMatrix"] >> cameraMatrix;
  fs_calibration["distortionCoefficients"] >> dist_coeffs;
  fs_calibration.release();

  std::cout << "camera matrix aruco : " << cameraMatrix << std::endl << "distortion coeffs: " << dist_coeffs << std::endl;

  //		ir_params(0,0) = cameraMatrix.at<double>(0,0);
  //		ir_params(1,1) = cameraMatrix.at<double>(1,1);
  //		ir_params(0,2) = cameraMatrix.at<double>(0,2);
  //		ir_params(1,2) = cameraMatrix.at<double>(1,2);

  //		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  //		cameraMatrix.at<double>(0,0) = 1057.932708427644;
  //		cameraMatrix.at<double>(1,1) = 1061.07748780408;
  //		cameraMatrix.at<double>(0,2) = 975.901271112996;
  //		cameraMatrix.at<double>(1,2) = 524.072951512137;
  //		dist_coeffs = (cv::Mat_<double>(5,1) <<  0.0548992182401387,-0.0805160815549322,
  //				-0.000938325112541061, 0.00181830067859300, 0.0290103875818698);
  //		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ aruco param. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  double arucoSquareDimension = 0.60; // in meters
  std::vector<int> markerIds; // markerIds.size() = number of Ids found
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
  //		params->doCornerRefinement = true;
  params->cornerRefinementWinSize = 5;
  params->cornerRefinementMaxIterations = 2000;
  params->adaptiveThreshConstant = true;
  params->cornerRefinementMinAccuracy = 0.001f;
  std::vector<cv::Vec3d> rotationVectors, translationVectors;
  cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Marker Detection ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  //	r_mutex.lock();
  cv::aruco::detectMarkers(inputImage,markerDictionary,markerCorners,markerIds, params);
  cv::aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, dist_coeffs, rotationVectors, translationVectors);
  //		if (markerIds.size() != number_of_aruco)
  //		{
  //			return;
  //		}
  //	r_mutex.unlock();
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Marker Visualization ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  cv::aruco::drawDetectedMarkers(inputImage,markerCorners, markerIds);
  imshow(filename_inputPicture,inputImage);


  /* TODO - CREATE CUSTOM CROPPING LIKE THIS
                 * ROI by creating mask for the parallelogram
                Mat mask = cvCreateMat(480, 640, CV_8UC1);
                // Create black image with the same size as the original
                for(int i=0; i<mask.cols; i++)
                   for(int j=0; j<mask.rows; j++)
                       mask.at<uchar>(Point(i,j)) = 0;

                // Create Polygon from vertices
                vector<Point> ROI_Poly;
                approxPolyDP(ROI_Vertices, ROI_Poly, 1.0, true);

                // Fill polygon white
                fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0);

                // Create new image for result storage
                Mat imageDest = cvCreateMat(480, 640, CV_8UC3);

                // Cut out ROI and store it in imageDest
                image->copyTo(imageDest, mask);
                */

  transform4x4.clear(); // clear transformations

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop (new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < markerIds.size(); i++)
  {
//    if (markerIds[i] == 1)
//    {
      cv::aruco::drawAxis(inputImage, cameraMatrix, dist_coeffs, rotationVectors[i], translationVectors[i], 0.1);
      max4points(markerCorners[i], topx, topy, botx, boty, invalid_points);
      //				std::cout << "invalid points? " << markerCorners[i] << std::endl;

      if (!invalid_points)
      {
        cv::Rect myROI(botx,boty,topx-botx, topy-boty);
        cv::Mat croppedImage = inputImage(myROI);

        // TODO: change from defined clouds to a std::map with string and cloud
        if (markerIds[i] == 1) cv::imshow(filename_crop1,croppedImage);
        if (markerIds[i] == 13) cv::imshow(filename_crop2,croppedImage);
        if (markerIds[i] == 40) cv::imshow(filename_crop3,croppedImage);
//        cv::imshow(filename_crop1,croppedImage);
        cv::waitKey(30);

        cv::Mat depthMatCropped = inputDepth(myROI);
//        current_depthMat_A_crop1_ = inputDepth(myROI);

        std::vector<float> aruco_cornerpoints = {botx, boty, topx, topy};
        pointcloudFromDepthImage(inputDepth, ir_params, src_cloud_crop, true, aruco_cornerpoints, depthMatCropped);

        // TODO: change from defined clouds to a std::map with string and cloud
        if (markerIds[i] == 1) src_cloud_crop1_ = src_cloud_crop;
        if (markerIds[i] == 13) src_cloud_crop2_ = src_cloud_crop;
        if (markerIds[i] == 40) src_cloud_crop3_ = src_cloud_crop;

        Eigen::Matrix4f tmatTemp = Eigen::Matrix4f::Identity();
        createTransMatrix(rotationVectors[i],translationVectors[i], tmatTemp);

        transform4x4.insert (std::pair<int, Eigen::Matrix4f> (markerIds[i],tmatTemp ));
      }
    }
}

void arucoProcessor::getCroppedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  *cloud += *src_cloud_crop1_;
  *cloud += *src_cloud_crop2_;
  *cloud += *src_cloud_crop3_;
}

//pcl::PointCloud<pcl::PointXYZ>::Ptr arucoProcessor::getCroppedCloud()
//{
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//  *cloud += *src_cloud_crop1_;
//  *cloud += *src_cloud_crop2_;
//  *cloud += *src_cloud_crop3_;
//  return cloud;
//}

void arucoProcessor::createTransMatrix(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, Eigen::Matrix4f& tMat)
{
  // Create the transformation matrix
  cv::Mat rotation3x3 = cv::Mat::eye(3, 3, CV_64F);
  cv::Rodrigues(rotationVectors, rotation3x3);
  tMat(0,0) = rotation3x3.at<double>(0,0);
  tMat(1,0) = rotation3x3.at<double>(1,0);
  tMat(2,0) = rotation3x3.at<double>(2,0);
  tMat(0,1) = rotation3x3.at<double>(0,1);
  tMat(1,1) = rotation3x3.at<double>(1,1);
  tMat(2,1) = rotation3x3.at<double>(2,1);
  tMat(0,2) = rotation3x3.at<double>(0,2);
  tMat(1,2) = rotation3x3.at<double>(1,2);
  tMat(2,2) = rotation3x3.at<double>(2,2);

  tMat(0,3) = translationVectors.val[0]*1.0f;
  tMat(1,3) = translationVectors.val[1]*1.0f;
  tMat(2,3) = translationVectors.val[2]*1.0f;

}

} // end namespace wp3

/*
 * functions.cpp
 *
 *  Created on: Sep, 2018
 *      Author: Atle Aalerud
 */

#include "wp3_calibrator/functions.h"

namespace wp3 {


void max4points(std::vector<cv::Point2f> cornerPoints, float & topx, float & topy, float & botx, float & boty, bool &flag)
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


void pointcloudFromDepthImage (cv::Mat& depth_image,
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



} // end namespace wp3

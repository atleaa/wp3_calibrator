#include "wp3_calibrator/visualization.h"

namespace wp3 {

// Constructor
Visualization::Visualization() :
  //    viewer_(new pcl::visualization::PCLVisualizer("Input clouds and registration"))
  viewer_("Input clouds and registration")
{

}


// Desctructor
Visualization::~Visualization()
{
  // do nothing
}


void Visualization::initialize()
{
  //  viewer_.generateColors(6)
  generateColors(6);
  std::cout << std::endl << "Created colors:" << std::endl;
  for (std::vector<cv::Vec3i>::const_iterator i = colors_.begin(); i != colors_.end(); ++i)
    std::cout << *i << ' ' << std::endl;
  std::cout << std::endl;

  viewer_.createViewPort (0.0, 0.0, 0.33, 0.5, vp11_);
  viewer_.createViewPort (0.33, 0.0, 0.66, 0.5, vp12_);
  viewer_.createViewPort (0.66, 0.0, 1.0, 0.5, vp13_);
  viewer_.createViewPort (0.0, 0.5, 0.33, 1.0, vp21_);
  viewer_.createViewPort (0.33, 0.5, 0.66, 1.0, vp22_);
  viewer_.createViewPort (0.66, 0.5, 1.0, 1.0, vp23_);
  viewer_.setBackgroundColor (0, 0, 0);
  viewer_.addText ("ARUCO CROPPED", 10, 10, "vp11_text", vp11_);
  viewer_.addText ("ICP1 CROPPED", 10, 10, "vp12_text", vp12_);
  viewer_.addText ("ICP2 CROPPED", 10, 10, "vp13_text", vp13_);
  viewer_.addText ("ARUCO FULL", 10, 10, "vp21 text", vp21_);
  viewer_.addText ("ICP1 FULL", 10, 10, "vp22 text", vp22_);
  viewer_.addText ("ICP2 FULL", 10, 10, "vp23 text", vp23_);

  //  viewer_.initCameraParameters();

  //Cam:
  // - pos: (-1.23666, -8.81802, -6.55671)
  // - view: (0.323175, -0.832741, 0.449555)
  //  - focal: (-0.0567675, -2.09815, 5.04277
  viewer_.setCameraPosition(-1.23666, -8.81802, -6.55671,
                            -0.0567675, -2.09815, 5.04277,
                            0.323175, -0.832741, 0.449555);
  viewer_.spinOnce();
}


void Visualization::initializeSingle()
{
  //  viewer_.generateColors(6)
  generateColors(6);
  std::cout << std::endl << "Created colors:" << std::endl;
  for (std::vector<cv::Vec3i>::const_iterator i = colors_.begin(); i != colors_.end(); ++i)
    std::cout << *i << ' ' << std::endl;
  std::cout << std::endl;

  viewer_.setBackgroundColor (0, 0, 0);
    //  viewer_.initCameraParameters();

  //Cam:
  // - pos: (-1.23666, -8.81802, -6.55671)
  // - view: (0.323175, -0.832741, 0.449555)
  //  - focal: (-0.0567675, -2.09815, 5.04277
  viewer_.setCameraPosition(-1.23666, -8.81802, -6.55671,
                            -0.0567675, -2.09815, 5.04277,
                            0.323175, -0.832741, 0.449555);
  viewer_.spinOnce();
}


void Visualization::generateColors(int colors_number)
{


  //  std::string camera_name = detections_vector[0].getSource()->getFrameId();
  //  if (strcmp(camera_name.substr(0,1).c_str(), "/") == 0)
  //  {
  //    camera_name = camera_name.substr(1, camera_name.size() - 1);
  //  }

  //  // Define color:
  //  int color_index;
  //  std::map<std::string, int>::iterator colormap_iterator = color_map.find(camera_name);
  //  if (colormap_iterator != color_map.end())
  //  { // camera already present
  //    color_index = colormap_iterator->second;
  //  }
  //  else
  //  { // camera not present
  //    color_index = color_map.size();
  //    color_map.insert(std::pair<std::string, int> (camera_name, color_index));
  //  }
  // -----------------------------------------

  colors_.push_back(cv::Vec3i(255, 0, 0)); // 1 is red
  colors_.push_back(cv::Vec3i(0, 255, 0)); // 2 is green
  colors_.push_back(cv::Vec3i(0, 0, 255)); // 3 is blue
  colors_.push_back(cv::Vec3i(255, 0, 255));
  colors_.push_back(cv::Vec3i(255, 255, 0));
  colors_.push_back(cv::Vec3i(0, 255, 255));
  //      std::cout << "vector size: " << colors.size() << std::endl;
  int predefined = colors_.size();
  for (unsigned int i = 0; i < colors_number-predefined; i++)
  {
    colors_.push_back(cv::Vec3i( (rand() % 256), (rand() % 256), (rand() % 256) )); // random color 0-255
    //        float(rand() % 256) / 255,
    //        float(rand() % 256) / 255,
    //        float(rand() % 256) / 255));
  }
}


//void Visualization::run(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
void Visualization::run(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_1,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_2,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_3,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_4,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_5,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_6,
                        std::map<std::string, Eigen::Matrix4f> & transformationsMap)
//                        std::vector<Eigen::Matrix4f> & transformVec)
//void Visualization::run(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector)
{
  //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_1(cloud, 0, 255, 0);
  viewer_.removeAllPointClouds();
  viewer_.removeAllShapes();
  //    viewer_.removeCoordinateSystem(vp11_);
  //    viewer_.removeCoordinateSystem(vp21_);

  viewer_.spinOnce(); // spin to clear


  //    for(std::map<std::string, int>::iterator it = map_.begin(); it != map_.end(); it++)
  //    {
  //      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb((cloud_vector[it->second]));
  //      viewer_name.addPointCloud<pcl::PointXYZRGB> (cloud_vector[it->second], rgb, it->first, v1);
  //      viewer_name.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, it->first, v1);
  //      viewer_name.spinOnce();
  //    }
  //    viewer_name.addText ("BEFORE CALIBRATION REFINEMENT", 10, 10, "v1 text", v1);

  //for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator cloud_itr = cloud_vector.begin(); cloud_itr != cloud_vector.end(); cloud_itr++)
  for(int i = 0; i < cloud_vector_1.size(); i++)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_curr(cloud_vector_1[i], colors_[i][0], colors_[i][1], colors_[i][2]);
    std::ostringstream cloud_stream;
    cloud_stream << "Node" << i << "_aruco_cropped";
    std::string cloud_str = cloud_stream.str();
    viewer_.addPointCloud<pcl::PointXYZ> (cloud_vector_1[i], color_curr, cloud_str,vp11_);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_str); //can add viewport
  }

  for(int i = 0; i < cloud_vector_2.size(); i++)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_curr(cloud_vector_2[i], colors_[i][0], colors_[i][1], colors_[i][2]);
    std::ostringstream cloud_stream;
    cloud_stream << "Node" << i << "_icp1_cropped";
    std::string cloud_str = cloud_stream.str();
    viewer_.addPointCloud<pcl::PointXYZ> (cloud_vector_2[i], color_curr, cloud_str,vp12_);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_str); //can add viewport
  }

  for(int i = 0; i < cloud_vector_3.size(); i++)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_curr(cloud_vector_3[i], colors_[i][0], colors_[i][1], colors_[i][2]);
    std::ostringstream cloud_stream;
    cloud_stream << "Node" << i << "_icp2_cropped";
    std::string cloud_str = cloud_stream.str();
    viewer_.addPointCloud<pcl::PointXYZ> (cloud_vector_3[i], color_curr, cloud_str,vp13_);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_str); //can add viewport
  }

  for(int i = 0; i < cloud_vector_4.size(); i++)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_curr(cloud_vector_4[i], colors_[i][0], colors_[i][1], colors_[i][2]);
    std::ostringstream cloud_stream;
    cloud_stream << "Node" << i << "_aruco";
    std::string cloud_str = cloud_stream.str();
    viewer_.addPointCloud<pcl::PointXYZ> (cloud_vector_4[i], color_curr, cloud_str,vp21_);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_str); //can add viewport
  }

  for(int i = 0; i < cloud_vector_5.size(); i++)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_curr(cloud_vector_5[i], colors_[i][0], colors_[i][1], colors_[i][2]);
    std::ostringstream cloud_stream;
    cloud_stream << "Node" << i << "_icp1";
    std::string cloud_str = cloud_stream.str();
    viewer_.addPointCloud<pcl::PointXYZ> (cloud_vector_5[i], color_curr, cloud_str,vp22_);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_str); //can add viewport
  }

  for(int i = 0; i < cloud_vector_6.size(); i++)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_curr(cloud_vector_6[i], colors_[i][0], colors_[i][1], colors_[i][2]);
    std::ostringstream cloud_stream;
    cloud_stream << "Node" << i << "_icp2";
    std::string cloud_str = cloud_stream.str();
    viewer_.addPointCloud<pcl::PointXYZ> (cloud_vector_6[i], color_curr, cloud_str,vp23_);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_str); //can add viewport
  }
  //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_1(cloud_vector[cloud_itr], 0, 255, 0);

  //    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name1",vp11_);
  //    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name2",vp12_);
  //    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name3",vp13_);
  //    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name4",vp21_);
  //    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name5",vp22_);
  //    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name6",vp23_);
  //    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name1"); //can add viewport
  //    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name2"); //can add viewport
  //    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name3"); //can add viewport
  //    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name4"); //can add viewport
  //    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name5"); //can add viewport
  //    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name6"); //can add viewport

  // add coordinates for given transforms
  //    for(int i = 0; i < transformVec.size(); i++)
  //    {
  ////    Eigen::Matrix4f a;
  //    Eigen::Affine3f Affine;
  //    Affine.matrix() = transformVec[i];
  //    viewer_.addCoordinateSystem (0.5, Affine);
  //    }

  //    std::map<std::string, std::vector<Eigen::Matrix4f>>::it
  //for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)

  for(std::map<std::string, Eigen::Matrix4f>::iterator transformation_iterator = transformationsMap.begin(); transformation_iterator != transformationsMap.end(); transformation_iterator++)
  {
    //        transformationsMap
    Eigen::Matrix4f tmpMatrix = transformation_iterator->second;
    Eigen::Affine3f tmpAffine;
    tmpAffine.matrix() = tmpMatrix;
    std::string tmpId;

    tmpId = std::string(transformation_iterator->first+"_vp11");
    //      if(!viewer_.updateCoordinateSystemPose(tmpId, tmpAffine))
    viewer_.removeCoordinateSystem(tmpId, vp11_);
    viewer_.addCoordinateSystem (1.0, tmpAffine, tmpId,vp11_);

    tmpId = std::string(transformation_iterator->first+"_vp21");
    //      if(!viewer_.updateCoordinateSystemPose(tmpId, tmpAffine))
    viewer_.removeCoordinateSystem(tmpId, vp21_);
    viewer_.addCoordinateSystem (1.0, tmpAffine, tmpId, vp21_);


    pcl::PointXYZ point;
    point.x = tmpMatrix(0,3);
    point.y = tmpMatrix(1,3);
    point.z = tmpMatrix(2,3);

    // (const std::string &text, const PointT &position, double textScale=1.0, double r=1.0, double g=1.0, double b=1.0, const std::string &id="", int viewport=0)
    //      viewer_.addText3D(transformation_iterator->first, point, 0.5, 1.0, 1.0, 1.0, transformation_iterator->first, vp11_);

    //BUG: add +1 to viewport due to bug in pcl 1.7, https://github.com/PointCloudLibrary/pcl/issues/1803
    viewer_.addText3D(transformation_iterator->first, point, 0.2, 1.0, 1.0, 1.0, transformation_iterator->first, vp11_+1);
    viewer_.addText3D(transformation_iterator->first, point, 0.2, 1.0, 1.0, 1.0, transformation_iterator->first, vp21_+1);
    //      //        transformationsMap
    //      Eigen::Affine3f Affine;
    //      Affine.matrix() = transformVec[0];
    //      if(!viewer_.updateCoordinateSystemPose("stringId", Affine))
    //      {
    //        viewer_.addCoordinateSystem (0.5, Affine, "stringId",vp11_);
    //        viewer_.addCoordinateSystem (0.5, Affine, "stringId",vp21_);
    //      }
  }




  //  viewer_->initCameraParameters ();

  //viewer_.updateText("ARUCO FULL VIEW UPDATED", 10, 10, "vp11_text");
  //viewer_.updateText("ARUCO CROPPED VIEW", 10, 10, "vp2 text");
  viewer_.spinOnce();
}

void Visualization::runSingle(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector,
                              std::map<std::string, Eigen::Matrix4f> & transformationsMap)
{
  viewer_.removeAllPointClouds();
  viewer_.removeAllShapes();
  viewer_.spinOnce(); // spin to clear


  for(int i = 0; i < cloud_vector.size(); i++)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_curr(cloud_vector[i], colors_[i][0], colors_[i][1], colors_[i][2]);
    std::ostringstream cloud_stream;
    cloud_stream << "Node" << i << "_aruco";
    std::string cloud_str = cloud_stream.str();
    viewer_.addPointCloud<pcl::PointXYZ> (cloud_vector[i], color_curr, cloud_str);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_str); //can add viewport
  }

  //    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name3"); //can add viewport


  for(std::map<std::string, Eigen::Matrix4f>::iterator transformation_iterator = transformationsMap.begin(); transformation_iterator != transformationsMap.end(); transformation_iterator++)
  {
    //        transformationsMap
    Eigen::Matrix4f tmpMatrix = transformation_iterator->second;
    Eigen::Affine3f tmpAffine;
    tmpAffine.matrix() = tmpMatrix;
    std::string tmpId;

    tmpId = std::string(transformation_iterator->first);
    //      if(!viewer_.updateCoordinateSystemPose(tmpId, tmpAffine))
    viewer_.removeCoordinateSystem(tmpId);
    viewer_.addCoordinateSystem (0.3, tmpAffine, tmpId);

    pcl::PointXYZ point;
    point.x = tmpMatrix(0,3);
    point.y = tmpMatrix(1,3);
    point.z = tmpMatrix(2,3);

    //BUG: add +1 to viewport due to bug in pcl 1.7, https://github.com/PointCloudLibrary/pcl/issues/1803
//    viewer_.addText3D(transformation_iterator->first, , 0.2, 1.0, 1.0, 1.0, transformation_iterator->first);
    viewer_.addText3D(transformation_iterator->first, point, 0.05, 1.0, 1.0, 1.0, transformation_iterator->first);

  }




  //  viewer_->initCameraParameters ();

  //viewer_.updateText("ARUCO FULL VIEW UPDATED", 10, 10, "vp11_text");
  //viewer_.updateText("ARUCO CROPPED VIEW", 10, 10, "vp2 text");
  viewer_.spinOnce();
}

//void
//TrajectoryRegistration::visualizeFinalRegistration (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& cloud_vector,
//    std::map<std::string, Eigen::Matrix4d>& registration_matrices, pcl::visualization::PCLVisualizer& viewer_name)
//{
//int v1(0);
//viewer_name.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//for(std::map<std::string, int>::iterator colormap_iterator = color_map_.begin(); colormap_iterator != color_map_.end(); colormap_iterator++)
//{
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb((cloud_vector[colormap_iterator->second]));
//  viewer_name.addPointCloud<pcl::PointXYZRGB> (cloud_vector[colormap_iterator->second], rgb, colormap_iterator->first, v1);
//  viewer_name.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, colormap_iterator->first, v1);
//  viewer_name.spinOnce();
//}
//viewer_name.addText ("BEFORE CALIBRATION REFINEMENT", 10, 10, "v1 text", v1);
//}

void Visualization::update()
{
  if (not viewer_.wasStopped()) //reg_viewer_???
  {
    //Save the position of the camera
    std::vector<pcl::visualization::Camera> cam;
    viewer_.getCameras(cam);

    std::ostringstream text;
    text << "Cam: " << endl
         << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl
                                                                                                 << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl
                                                                                                                                                                                              << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;
    std::string text_str = text.str();

    //    cout << text; //Print recorded points on the screen:

    if(not viewer_.updateText(text_str, 10, 30, "Cam_text"))
    {
      viewer_.addText(text_str, 10, 30, "Cam_text");
    }

    viewer_.spinOnce();
    //    cv::waitKey(1);
  }
}

} // end namespace wp3

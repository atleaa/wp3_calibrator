#include "wp3_calibrator/visualization.h"

namespace wp3 {

// contstructor
Visualization::Visualization() :
    //    viewer_(new pcl::visualization::PCLVisualizer("Input clouds and registration"))
    viewer_("Input clouds and registration")
{
    //viewer_ = new pcl::visualization::PCLVisualizer("Input clouds and registration");
    viewer_.createViewPort (0.0, 0.0, 0.33, 0.5, vp11);
    viewer_.createViewPort (0.33, 0.0, 0.66, 0.5, vp12);
    viewer_.createViewPort (0.66, 0.0, 1.0, 0.5, vp13);
    viewer_.createViewPort (0.0, 0.5, 0.33, 1.0, vp21);
    viewer_.createViewPort (0.33, 0.5, 0.66, 1.0, vp22);
    viewer_.createViewPort (0.66, 0.5, 1.0, 1.0, vp23);
    viewer_.setBackgroundColor (0, 0, 0);
    //  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_2(cloud2, 0, 255, 255);
    viewer_.addText ("ARUCO FULL VIEW", 10, 10, "vp11_text", vp11);
    viewer_.addText ("ARUCO CROPPED VIEW", 10, 10, "vp2 text", vp21);

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

Visualization::~Visualization()
{
    // do nothing
}

void Visualization::run(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_1(cloud, 0, 255, 0);
    viewer_.removeAllPointClouds();
    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name1",vp11);
    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name2",vp12);
    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name3",vp13);
    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name4",vp21);
    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name5",vp22);
    viewer_.addPointCloud<pcl::PointXYZ> (cloud, color_1, "cloud_name6",vp23);
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name1"); //can add viewport
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name2"); //can add viewport
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name3"); //can add viewport
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name4"); //can add viewport
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name5"); //can add viewport
    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_name6"); //can add viewport
    viewer_.addCoordinateSystem (1.0);
    //  viewer_->initCameraParameters ();

    //viewer_.updateText("ARUCO FULL VIEW UPDATED", 10, 10, "vp11_text");
    //viewer_.updateText("ARUCO CROPPED VIEW", 10, 10, "vp2 text");
    viewer_.spinOnce();
}


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

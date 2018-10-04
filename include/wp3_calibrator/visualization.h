#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <string.h>

namespace wp3 {

class Visualization
{
public:
    // Contructor
    Visualization();

    // Deconstrucor
    ~Visualization();

    void initialize();

    void initializeSingle();

    void generateColors(int colors_number);

    // Run cloud visualization
//    void run(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
//    void run(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector);
    void run(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_1,
             std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_2,
             std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_3,
             std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_4,
             std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_5,
             std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector_6,
             std::map<std::string, Eigen::Matrix4f> &transformationsMap);
//             std::vector<Eigen::Matrix4f> & transformVec);
//    void run(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector);

    void runSingle(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >& cloud_vector,
             std::map<std::string, Eigen::Matrix4f> &transformationsMap);

    // Spin and update pose
    void update();

private:
//    pcl::visualization::PCLVisualizer& viewer_;
//    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pcl::visualization::PCLVisualizer viewer_;
    std::vector<cv::Vec3i> colors_;     // vector containing colors to use to identify cameras in the network
    int vp11_;
    int vp12_;
    int vp13_;
    int vp21_;
    int vp22_;
    int vp23_;

    /** \brief Map between camera frame_id and color */
    std::map<std::string, int> map_;
};

}
#endif // VISUALIZATION_H

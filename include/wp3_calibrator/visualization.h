#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/highgui/highgui.hpp>
namespace wp3 {

class Visualization
{
public:
    // Contructor
    Visualization();

    // Deconstrucor
    ~Visualization();

    // Run cloud visualization
    void run(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // Spin and update pose
    void update();

private:
//    pcl::visualization::PCLVisualizer& viewer_;
//    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pcl::visualization::PCLVisualizer viewer_;
    int vp11;
    int vp12;
    int vp13;
    int vp21;
    int vp22;
    int vp23;
};

}
#endif // VISUALIZATION_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // 填充数据
    cloud.width = 180;
    cloud.height = 16;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (auto &point : cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("../test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("../test_pcd.pcd", cloud)== -1)
    {
        std::cout << "NO such file" << std::endl;
    }
    else
    {
        std::cout << "Load the file" << std::endl;
    }
    return (0);
}
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);

}
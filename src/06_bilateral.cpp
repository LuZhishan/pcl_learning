#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/impl/bilateral.hpp>

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::io::loadPCDFile("../table.pcd", *source_cloud);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::BilateralFilter<pcl::PointXYZI> bf_filter;
    bf_filter.setInputCloud(source_cloud);
    bf_filter.setSearchMethod(tree);
    bf_filter.setStdDev(0.1);
    bf_filter.setHalfSize(0.1);
    bf_filter.filter(*filtered_cloud);

    pcl::visualization::PCLVisualizer viewer("Title of Windows");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_of_point(filtered_cloud, 255, 0, 0);
    viewer.addPointCloud(filtered_cloud, color_of_point, "ID");
    while (viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
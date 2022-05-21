#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/fpfh.h>
#include <pcl/visualization/pcl_plotter.h> //可视化直方图

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(source_cloud);
    fpfh.setInputNormals(cloud_normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.5);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_point(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.compute(*fpfh_point);

    pcl::visualization::PCLPlotter poltter;
    poltter.addFeatureHistogram(*fpfh_point, 300);
    poltter.plot();
    return 0;
}
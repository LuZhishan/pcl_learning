#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/pfh.h>
#include <pcl/visualization/pcl_plotter.h> //可视化直方图

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(source_cloud);
    pfh.setInputNormals(cloud_normals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pfh.setSearchMethod(tree);
    pfh.setRadiusSearch(0.5);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_point(new pcl::PointCloud<pcl::PFHSignature125>);
    pfh.compute(*pfh_point);

    pcl::visualization::PCLPlotter poltter;
    poltter.addFeatureHistogram(*pfh_point, 300);
    poltter.plot();
    return 0;
}
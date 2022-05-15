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

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(source_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03);   // 搜索半径，单位：米
    ne.compute(*cloud_normals);


    pcl::visualization::PCLVisualizer viewer("Title of Windows");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal>  color_red(cloud_normals, 0, 0, 255);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(source_cloud, cloud_normals);
    viewer.spin();
}

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

//利用积分图估计法线，这里把点云看作深度图
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h> // 仅适用于有序点云

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ine;
    //三种法线估计方法
        // COVARIANCE_MATRIX
        // AVERAGE_3D_GRADIENT
        // AVERAGE_DEPTH_CHANGE
    ine.setNormalEstimationMethod(ine.AVERAGE_3D_GRADIENT);
    ine.setMaxDepthChangeFactor(0.02f); // 计算对象边界的深度变化阈值
    ine.setNormalSmoothingSize(10.0f);  // 
    ine.setInputCloud(source_cloud);
    ine.compute(*normals);

    pcl::visualization::PCLVisualizer viewer("Title of Windows");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal>  color_red(normals, 0, 0, 255);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(source_cloud, normals);
    viewer.spin();
}
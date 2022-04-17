#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> //坐标转换
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../test_pcd.pcd", *source_cloud);

    Eigen::Matrix4f tf_1 = Eigen::Matrix4f::Identity();
    tf_1(0, 0) = cos(M_PI_4); //pi/4, 也就是45°
    tf_1(0, 1) = -sin(M_PI_4);
    tf_1(1, 0) = sin(M_PI_4);
    tf_1(1, 1) = cos(M_PI_4);
    tf_1(0, 3) = 2.5;//x方向移动2.5
    std::cout << tf_1 << std::endl << std::endl;


    Eigen::Affine3f tf_2 = Eigen::Affine3f::Identity();
    tf_2.translation() << 2.5, 0, 0; //x方向移动2.5
    tf_2.rotate(Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()));//绕z轴旋转45°
    std::cout << tf_2.matrix() << std::endl << std::endl;

    pcl::transformPointCloud(*source_cloud, *transformed_cloud, tf_2);

    pcl::visualization::PCLVisualizer viewer("Title of Windows");

    int v1(0), v2(0);
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.setBackgroundColor(0.1, 0.1, 0.1, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_blue(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  color_red(transformed_cloud, 0, 0, 255);
    viewer.addPointCloud(source_cloud, color_blue, "v1", v1);
    viewer.addPointCloud(transformed_cloud, color_red , "v2", v2);
    viewer.spin();

    return 0;
}
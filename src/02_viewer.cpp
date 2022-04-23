#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../test_pcd.pcd", *cloud);

    pcl::visualization::PCLVisualizer viewer("Title of Windows");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_of_point(cloud, 255, 0, 0);
    viewer.addPointCloud(cloud, color_of_point, "ID");
    // viewer.spin();

    int v1(0), v2(0);
    viewer.createViewPort(0, 0, 0.5, 1, v1);//(x_min, y_min, x_max, y_max,viewPort)
    viewer.createViewPort(0.5, 0, 1, 1, v2);//两个窗口左右并排
    viewer.setBackgroundColor(0, 0, 0, v1); //设置背景色
    viewer.setBackgroundColor(0.1, 0.1, 0.1, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_blue(cloud, 255, 0, 0); //设置点云的颜色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  color_red(cloud, 0, 0, 255);
    viewer.addPointCloud(cloud, color_blue, "v1", v1);
    viewer.addPointCloud(cloud, color_red , "v2", v2);
    // viewer.spin();
    while (viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return (0);
}
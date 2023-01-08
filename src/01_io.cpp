#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("../table.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }
    // 最简单的可视化
    pcl::visualization::PCLVisualizer viewer("Title of windows");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_of_cloud(cloud_in, 255,0,0);
    viewer.addPointCloud(cloud_in, color_of_cloud, "ID");
    // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_in, normal, 10, 1, "normals"); // 可视化法线
    // viewer.spin();

    // 可以添加的选项
    viewer.addCoordinateSystem(1, "ID", 0); // 添加ID点云的坐标系
    viewer.setBackgroundColor(0.1, 0.1, 0.1); // 设置背景，0为黑，1为白
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ID"); // 设置点云可视化时的大小
    viewer.spin();

}
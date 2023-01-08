#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("../table.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(50);           // 数目阈值，低于此值将被判定为离群点
    sor.setStddevMulThresh(1.5);// 距离阈值，在平均距离标准差的1.5倍距离上有没有50个点
    sor.filter(*cloud_s);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_in);
    ror.setRadiusSearch(0.05);       // 距离阈值，0.2米
    ror.setMinNeighborsInRadius(20);// 数目阈值
    ror.filter(*cloud_r);

    pcl::visualization::PCLVisualizer viewer("Title of window");
    int v1(1), v2(2);
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_r(cloud_s, 255,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_g(cloud_s, 0,255,0);
    viewer.addPointCloud(cloud_s, color_r, "s", v1);
    viewer.addPointCloud(cloud_r, color_g, "r", v2);
    viewer.spin();

    return 0;
}
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>

// 去地面，提取地面点效果很好，非地面点效果很差，由很多非地面点被删除了

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("/home/fanliang/12345/pcd/ouster.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }

    pcl::PointIndices::Ptr ground_index(new pcl::PointIndices);
    pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud_in);
    pmf.setMaxWindowSize(20);   // 最大窗口尺寸（判断是否停止迭代）
    pmf.setSlope(1);            // 坡度
    pmf.setInitialDistance(0.5);// 初始高程差阈值
    pmf.setMaxDistance(1.5);    // 最大高程差
    pmf.extract(ground_index->indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(ground_index);
    extract.setNegative(false);
    extract.filter(*cloud_ground);
    extract.setNegative(true);
    extract.filter(*cloud_no_ground);

    pcl::visualization::PCLVisualizer viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_ground(cloud_ground, 255,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_no_ground(cloud_ground, 0,255,0);
    viewer.addPointCloud(cloud_ground, color_ground, "ground");
    viewer.addPointCloud(cloud_no_ground, color_no_ground, "no_ground");// 这里可以看出非地面点的效果很差
    viewer.spin();
}
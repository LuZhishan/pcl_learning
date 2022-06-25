#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/model_types.h>   // 模型定义
#include <pcl/sample_consensus/method_types.h>  // 随机参数估计方法
#include <pcl/segmentation/sac_segmentation.h>  // 采样一致性分割算法

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);

    pcl::ModelCoefficients::Ptr coe(new pcl::ModelCoefficients);// 存储平面模型系数
    pcl::PointIndices::Ptr inlers(new pcl::PointIndices);       // 存储平面点
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);      // 设置优化系数
    seg.setModelType(pcl::SACMODEL_PLANE);  // 使用SAC算法获取平面
    seg.setMethodType(pcl::SAC_RANSAC);     // 随机一致取样算法
    seg.setDistanceThreshold(0.01);         // 距离阈值，单位： 米。就是说距离平面1厘米内的点算平面点
    seg.setInputCloud(source_cloud);
    seg.segment(*inlers, *coe);

    for(int i = 0; i < 4; i++)
    {
        std::cout << coe->values[i] << std::endl;
    }
}
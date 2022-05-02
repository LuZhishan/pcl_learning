#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);

    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients); //创建
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;    //模型分割对象
    seg.setOptimizeCoefficients(true);          //对模型参数进行优化
    seg.setModelType(pcl::SACMODEL_PLANE);      //优化模型
    seg.setMaxIterations(1000);                 //最大迭代次数
    seg.setDistanceThreshold(0.01);             //距离阈值

    pcl::ExtractIndices<pcl::PointXYZ> ei_filter;//创建点云提取对象
    int i = 0, point_num = (int) source_cloud->size();
    while (source_cloud->size() > point_num)
    {
        seg.setInputCloud(source_cloud);
        seg.segment(*inliers, *coef);
        if(inliers->indices.size() == 0)
        {
            break;
        }
        ei_filter.setInputCloud(source_cloud);
        ei_filter.setIndices(inliers);
        ei_filter.setNegative(false);
        ei_filter.filter(*filtered_cloud);
    }
    




    ;
    

}
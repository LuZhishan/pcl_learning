#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// ransac提取模型及其汇总
// SACMODEL_PLANE，[normal.x, normal.y, normal.z, d]，平面方程 ax + by + cz + d=0，abc就是平面法向量
// SACMODEL_LINE， [point_x, point_y, point_z, direction_a, direction_b, direction_c] 
                // 直线方程 (x - x0)/a = (y - y0)/b = (z - z0)/c，abc是直线的方向向量
// SACMODEL_CIRCLE2D，[center.x, center.y, radius]
// SACMODEL_CIRCLE3D，[center.x, center.y, center.z, radius, normal.x, normal.y, normal.z]
// SACMODEL_SPHERE，[center.x, center.y, center.z, radius]
// SACMODEL_CYLINDER，[point_on_axis.x, point_on_axis.y, point_on_axis.z, axis_direction.x, axis_direction.y, axis_direction.z, radius]

int main()
{
    srand(time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("../table.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coe(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);        // 设置阀值

    // 点云里可能有多个平面，所以加了一个循环
    int initial_point_size = cloud_in->points.size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer viewer;
    while (cloud_in->points.size() > initial_point_size * 0.3)
    {
        seg.setInputCloud(cloud_in);
        seg.segment(*inliers, *coe);
        if (inliers->indices.size() == 0)
        {
            std::cout << "No plane detected now" << std::endl;
            break;
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        std::cout << cloud_plane->points.size() << std::endl;

        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_plane, r,g,b);
        viewer.addPointCloud(cloud_plane, color, std::to_string(r));

        extract.setNegative(true);
        extract.filter(*cloud_temp);
        cloud_in = cloud_temp;        
    }
    viewer.spin();
    return 0;
}
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    srand(time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in->width = 100;
    cloud_in->height = 1;
    cloud_in->points.resize(100);
    for (size_t i = 0; i < 100; i++)
    {
        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    pcl::ModelCoefficients::Ptr coe(new pcl::ModelCoefficients);
    coe->values.resize(4);
    coe->values[0] = 0;
    coe->values[1] = 1;
    coe->values[2] = 2;
    coe->values[3] = 3;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setInputCloud(cloud_in);
    proj.setModelType(pcl::SACMODEL_PLANE); // 投影至平面
    proj.setModelCoefficients(coe);
    proj.filter(*cloud_out);

    pcl::visualization::PCLVisualizer viewer("Title of window");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_out, 255,0,0);
    viewer.addPointCloud(cloud_out, color, "ID");
    viewer.spin();
    
    return 0;
}
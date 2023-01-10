#include <pcl/io/pcd_io.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/visualization/pcl_visualizer.h>

// 根据相机视角进行裁剪

int main()
{
    srand(time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in->width = 5000;
    cloud_in->height = 1;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        cloud_in->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    // Frustum是一个砍头的四棱锥，NearPlane是上顶面，FarPlane是下底面
    pcl::FrustumCulling<pcl::PointXYZ> fc;
    fc.setInputCloud(cloud_in);
    fc.setVerticalFOV(45);
    fc.setHorizontalFOV(60);
    fc.setNearPlaneDistance(200);   // 上顶面
    fc.setFarPlaneDistance(1000);   // 下底面
    Eigen::Affine3f cam_pose = Eigen::Affine3f::Identity();
    cam_pose.translate(Eigen::Vector3f(512, 0, 512));
    cam_pose.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ()));
    fc.setCameraPose(cam_pose.matrix());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    fc.filter(*cloud_out);

    pcl::visualization::PCLVisualizer viewer("Title of windows");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_w(cloud_in, 255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_r(cloud_out, 255,0,0);
    viewer.addPointCloud(cloud_in, color_w, "in");
    viewer.addPointCloud(cloud_out, color_r, "out");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "out");
    viewer.spin();

    return 0;
}
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("../table.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }
    Eigen::Affine3d tf = Eigen::Affine3d::Identity();// 位置在原点，方向指向x轴
    tf.translate(Eigen::Vector3d(2.5, 0, 0));
    // tf.translation() << 2.5, 0, 0; // 等同于上一行
    tf.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0, 2, 3)));
    std::cout << tf.matrix() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf_ed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_in, *cloud_tf_ed, tf);

    pcl::visualization::PCLVisualizer viewer("Title of windows");
    int v1(1), v2(2);
    viewer.createViewPort(0, 0, 0.5, 1, v1);//(x_min, y_min, x_max, y_max, viewPort)
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    viewer.setBackgroundColor(0.1, 0.2, 0.3, v1);
    viewer.setBackgroundColor(0.3, 0.2, 0.1, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_r(cloud_in, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_g(cloud_tf_ed, 0, 255, 0);
    viewer.addPointCloud(cloud_in, color_r, "v1", v1);
    viewer.addPointCloud(cloud_tf_ed, color_g, "v2", v2);
    viewer.spin();

    return 0;
}
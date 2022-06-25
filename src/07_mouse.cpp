#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include  <boost/thread.hpp>
boost::mutex cloud_mutex;

struct callback_args
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_point;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};
void click_callback(const pcl::visualization::PointPickingEvent& event, void *input_args)
{
    struct callback_args* data = (struct callback_args *)input_args;
    if(event.getPointIndex() == -1)
    {
        return;
    }
    pcl::PointXYZ current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);  // 从鼠标事件中获取点的坐标存到current_point中
    data->clicked_point->points.push_back(current_point);               // 将该点存到data中

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->clicked_point, 255, 0, 0);
    data->viewer->removePointCloud("clicked_points");                   // 移除原来的点，并用指定大小的点覆盖
    data->viewer->addPointCloud(data->clicked_point, red, "clicked_points");
    data->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point << std::endl;
}

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../test_pcd.pcd", *cloud);

    // pcl::visualization::PCLVisualizer viewer("Title of Windows");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Title of Windows"));
    cloud_mutex.lock();
    viewer->addPointCloud(cloud, "pick");
    struct callback_args cb_args;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_point_3d(new pcl::PointCloud<pcl::PointXYZ>);
    cb_args.clicked_point = clicked_point_3d;
    cb_args.viewer = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(click_callback, (void *)&cb_args);
    viewer->spin();
    cloud_mutex.unlock();

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
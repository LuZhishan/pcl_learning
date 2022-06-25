#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <boost/thread/thread.hpp>
#include <pcl/features/narf_descriptor.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);

    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    range_image_ptr->createFromPointCloud(source_cloud, 0.2, 
            pcl::deg2rad(360.0), pcl::deg2rad(180.0),
            )

}
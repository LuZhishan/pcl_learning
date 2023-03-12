// #include <iostream>
// using namespace std;

// int main()
// {
//     cout << (25&255) << endl;
// }

// #include <pcl/io/pcd_io.h>

// int main()
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr my_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>* my_star;
//     my_star = my_ptr.get();

//     pcl::PointCloud<pcl::PointXYZ>* my_star2(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr my_ptr2;
//     my_ptr2.reset(my_star2);
// }

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

int main()
{
    AngleAxisd a(0.7, Vector3d(0,0,1));
    std::cout << a.matrix() << std::endl;
}
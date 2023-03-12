#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile("/home/fanliang/12345/pcd/xyzi.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }

    Eigen::Vector4f pca_centroid;   // 质心坐标(x,y,z,1)
    pcl::compute3DCentroid(*cloud_in, pca_centroid);
    std::cout << pca_centroid << std::endl;
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud_in, pca_centroid, covariance);
    std::cout << covariance << std::endl;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f pca_vectors = eigen_solver.eigenvectors();
    Eigen::Vector3f pca_values = eigen_solver.eigenvalues();
    std::cout << pca_values << std::endl;   // 特征值是从小到大排列
    std::cout << pca_vectors << std::endl;

    Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
    transform.block<3,3>(0,0) = pca_vectors.transpose();
    std::cout << transform.block<3,1>(0,3) << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tf_ed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud_in, *cloud_tf_ed, transform);

    pcl::visualization::PCLVisualizer viewer("Title of windows");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_of_cloud(cloud_tf_ed, 255,0,0);
    viewer.addPointCloud(cloud_tf_ed, color_of_cloud, "ID");
    viewer.spin();

}
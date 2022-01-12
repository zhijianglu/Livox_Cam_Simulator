#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>



typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char *argv[])
{
    PointCloudT::Ptr cloud(new pcl::PointCloud<PointT>);
    std::string file(argv[1]);
    if (pcl::io::loadPCDFile<PointT>(file, *cloud )== -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    std::cout << "Loaded "
              << cloud->width*cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    pcl::visualization::CloudViewer viewer("viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {

    }
    return 0;
}
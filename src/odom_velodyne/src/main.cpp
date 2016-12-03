#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>

#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>

namespace
{
pcl::PointCloud<pcl::PointXYZ>::Ptr PCxyzPrevious;

ros::Publisher pub;
}  // namespace

void callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
  // pcl::PCLPointCloud2 cloud_filtered;
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud(input);
  // sor.setLeafSize(0.01, 0.01, 0.01);
  // sor.filter(cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr PCxyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *PCxyz);

  if (PCxyzPrevious)
  {
    ROS_DEBUG_STREAM("Calculating transformation...");

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(PCxyz);
    icp.setInputTarget(PCxyzPrevious);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(1);
    // Perform the alignment
    pcl::PointCloud<pcl::PointXYZ> PCxyzAligned;
    icp.align(PCxyzAligned);
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transf4f = icp.getFinalTransformation();

    // ROS_INFO_STREAM("Transf4f: " << std::endl << transf4f);

    const Eigen::Affine3d transf3d(transf4f.cast<double>());
    geometry_msgs::Transform transform_msg;
    tf::transformEigenToMsg(transf3d, transform_msg);
    pub.publish(transform_msg);
  }
  else
  {
    ROS_INFO_STREAM("First cloud acquired.");
  }
  // Save the cloud for next iteration
  PCxyzPrevious = PCxyz;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("kinect_input", 1, callback);

  pub = nh.advertise<geometry_msgs::Transform>("transformation", 1);

  ROS_INFO_STREAM("Node initialized.");

  // Spin
  ros::spin();
  // return 0;
}

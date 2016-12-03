#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
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
using namespace Eigen;
using Namespace std;

ros::Time current_time, previous_time;
current_time = ros::Time::now();
previous_time = ros::Time::now();
ros::Rate r(1.0);
//initialize pointers to store (t-1) cloud
namespace
{
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous;

ros::Publisher pub;
ros::Publisher odom_pub;
}  // namespace

void callback(const sensor_msgs::PointCloud2ConstPtr &input)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud_current);

  if (cloud_previous)
  {
    ROS_DEBUG_STREAM("Starting registration");

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_current);
    icp.setInputTarget(cloud_previous);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(1);
    // Perform the alignment
    pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
    icp.align(cloud_aligned);
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f trans = icp.getFinalTransformation();

    ROS_INFO_STREAM("Transf4f: " << std::endl << trans);

     Matrix3f rot;
 for(int i = 0; i<3; i++)
{
   for(int j = 0; j<3; j++)
	{
		rot(i,j) = trans(i,j);
	}
}
Quaternionf q;
q = rot;
pose.pose.position.x = trans(0,3);
pose.pose.position.y = trans(1,3);
pose.pose.position.z = trans(2,3);

pose.pose.orientation.x = q.x();
pose.pose.orientation.y = q.y();
pose.pose.orientation.z = q.z();
pose.pose.orientation.w = q.w();


double x = 0.0;
   double y = 0.0;
     double th = 0.0;

     double vx = 0.0
     double vy = 0.0
double vth = 0.0


x = trans(0,3);
y = trans(1,3);
z = trans(2,3);
     current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    const Eigen::Affine3d transf3d(transf4f.cast<double>());
    geometry_msgs::Transform transform_msg;
    tf::transformEigenToMsg(transf3d, transform_msg);
    pub.publish(transform_msg);
    odom_pub.publish(odom);
  }
  else
  {
    ROS_INFO_STREAM("First cloud acquired.");
  }
  // Save the cloud for next iteration
  cloud_previous = cloud_current;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("velodyne_points", 1, callback);

  pub = nh.advertise<geometry_msgs::Transform>("transformation", 1);

  odom_pub = nh.advertise<nav_msgs>::Odometry>("odom,50")

  ROS_INFO_STREAM("Node initialized.");

  // Spin
  ros::spin();
  // return 0;
}

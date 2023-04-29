#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "icp/eigen_kdtree.h"
#include "map.h"
#include "normal_estimator.h"
#include "normal_localizer2d.h"
#include "ros_bridge.h"

const std::string TOPIC_SCAN = "base_scan";
const std::string TOPIC_INITIALPOSE = "initialpose";
const std::string TOPIC_ODOM = "odom_out";
const std::string TOPIC_MAP = "map";

// Map callback definition
void callback_map(const nav_msgs::OccupancyGridConstPtr&);
// Initial pose callback definition
void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
// Scan callback definition
void callback_scan(const sensor_msgs::LaserScanConstPtr&);

std::shared_ptr<Map> map_ptr = nullptr;
ros::Publisher pub_scan, pub_odom;

NormalLocalizer2D localizer;

int main(int argc, char** argv) {

  // Initialize ROS system
  ros::init(argc, argv, "localizer_node");

  // Create a NodeHandle to manage the node.
  // The namespace of the node is set to global
  ros::NodeHandle nh("/");

  // Create shared pointer for the Map object
  std::shared_ptr<Map> p(new Map);
  map_ptr=p;
  /*
   * Subscribe to the topics:
   * /map [nav_msgs::OccupancyGrid]
   * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
   * /base_scan [sensor_msgs::LaserScan]
   * and assign the correct callbacks
   */
  ros::Subscriber scan_subscriber = nh.subscribe(TOPIC_MAP, 30, callback_map);

   ros::Subscriber scan_subscriber2 = nh.subscribe(TOPIC_INITIALPOSE, 30, callback_initialpose);

   ros::Subscriber scan_subscriber3 = nh.subscribe(TOPIC_SCAN, 30, callback_scan);

  /*
   * Advertise the following topic:
   * /odom_out [nav_msgs::Odometry]
   */
  pub_odom = nh.advertise<nav_msgs::Odometry>(TOPIC_ODOM, 10);

  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);

  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();
  
  return 0;
}

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg_) {
  /* If the internal map is not initialized,
   * load the incoming occupancyGrid and
   * set the localizer map accordingly
   * Remember to load the map only once during the execution of the map.
  */
  std::cerr<< "START callback_map"<<std::endl;

  if (!map_ptr->initialized()){
    std::cerr<< "map initialized"<<std::endl;

    map_ptr->loadOccupancyGrid(msg_);
    
    localizer.setMap(map_ptr);
  }
    std::cerr<< "End callback_map"<<std::endl;

}

void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_) {
  /**
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   */

  geometry_msgs::Pose current_pose;
  current_pose = msg_->pose.pose;
  Eigen::Isometry2f iso;
  pose2isometry(current_pose,iso);
  localizer.setInitialPose(iso);

}

void callback_scan(const sensor_msgs::LaserScanConstPtr& msg_) {
  /**
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */
    NormalLocalizer2D::LaserContainerType laser_;
    scan2eigen(msg_,laser_);

  /**
   * Augment every point in scan with normals.
   * Use the NormalEstimator
   *
   */
  NormalEstimator n_est = NormalEstimator(laser_,40);
  NormalEstimator::NormalLaserScanType scan_with_normals;
  n_est.get(scan_with_normals);


  /**
   * Set the laser parameters and process the incoming scan through the
   * localizer
   */
  localizer.setLaserParams(msg_->range_min,msg_->range_max,msg_->angle_min,msg_->angle_max,msg_->angle_increment);


  /**
   * Send a transform message between FRAME_WORLD and FRAME_LASER.
   * The transform should contain the pose of the laser with respect to the map
   * frame.
   * You can use the isometry2transformStamped function to convert the isometry
   * to the right message type.
   * Look at include/constants.h for frame names
   *
   * The timestamp of the message should be equal to the timestamp of the
   * received message (msg_->header.stamp)
   */
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped trans_msg;
  std::string frame_id_;
  std::string child_frame_id_;
  ros::Time stamp_ = msg_->header.stamp;
  isometry2transformStamped(localizer.getCurrentLaserPose(),trans_msg,frame_id_,child_frame_id_,stamp_);



  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
  nav_msgs::Odometry odo_msg;
  transformStamped2odometry(trans_msg,odo_msg);
  pub_odom.publish(odo_msg);
  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);
}
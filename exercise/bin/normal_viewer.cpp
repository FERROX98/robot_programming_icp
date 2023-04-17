#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "normal_estimator.h"
#include "ros_bridge.h"

void scan_cb(const sensor_msgs::LaserScanConstPtr);

using Point2f = Eigen::Vector2f;
using ContainerPoint2f =
    std::vector<Point2f, Eigen::aligned_allocator<Point2f>>;
using PointNormal = Eigen::Vector4f;
using ContainerPointNormal =
    std::vector<PointNormal, Eigen::aligned_allocator<PointNormal>>;

int main(int argc, char** argv) {
  ros::init(argc, argv, "normal_estimator");

  ros::NodeHandle nh("/");

  ros::Subscriber scan_sub = nh.subscribe("/base_scan", 10, scan_cb);

  ros::spin();
  return 0;
}

float res = 0.01;
float ires = 1. / res;
cv::Size2i wsize(900, 900);

cv::Point2i toImageIdx(const PointNormal& p_, float ires_, cv::Size2i wsize_) {
  cv::Point2i ret;
  ret.x = (p_.x() * ires_) + wsize_.height / 2;
  ret.y = (p_.y() * ires_) + wsize_.width / 2;
  return ret;
}

void scan_cb(const sensor_msgs::LaserScanConstPtr msg_) {
  ContainerPoint2f scan;
  scan2eigen(msg_, scan);

  NormalEstimator ne(scan, 10);
  ContainerPointNormal scan_normal;
  ne.get(scan_normal);

  cv::Mat_<cv::Vec3b> viz_frame(wsize, cv::Vec3b::all(255));
  for (const auto p : scan_normal) {
    cv::Point2f p_coords = toImageIdx(p, ires, wsize);
    cv::Point2f p_vert = p_coords;
    p_vert.x += p.z() * 30;
    p_vert.y += p.w() * 30;
    cv::Vec3b color(255, 255 * (p(2) + 1) / 2, 255 * (p(3) + 1) / 2);
    cv::circle(viz_frame, p_coords, 3, color, -1);
    cv::arrowedLine(viz_frame, p_coords, p_vert, color);
  }
  cv::imshow("Viz", viz_frame);
  cv::waitKey(10);
}
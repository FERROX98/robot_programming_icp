#pragma once

#include <Eigen/Dense>
#include <vector>

#include "eigen_kdtree.h"

class NormalEstimator {
 public:
  using Vector2f = Eigen::Vector2f;
  using Vector4f = Eigen::Vector4f;
  using LaserScanType =
      std::vector<Vector2f, Eigen::aligned_allocator<Vector2f>>;
  using NormalLaserScanType =
      std::vector<Vector4f, Eigen::aligned_allocator<Vector4f>>;

  NormalEstimator(LaserScanType& src_, int max_points_in_leaf);
  inline void get(NormalLaserScanType& dest_) { dest_ = _scan_with_normals; }

 protected:
  using TreeNodeType = TreeNode_<typename LaserScanType::iterator>;
  TreeNodeType _kd_tree;
  NormalLaserScanType _scan_with_normals;
  int flg_log=0;
};
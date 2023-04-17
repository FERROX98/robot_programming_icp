#include "normal_estimator.h"

#include "eigen_covariance.h"

NormalEstimator::NormalEstimator(LaserScanType& src_, int max_points_in_leaf)
    : _kd_tree(src_.begin(), src_.end(), max_points_in_leaf) {
  /**
   * TODO
   */
}
#include "normal_estimator.h"

#include "eigen_covariance.h"

NormalEstimator::NormalEstimator(LaserScanType& src_, int max_points_in_leaf)
    : _kd_tree(src_.begin(), src_.end(), max_points_in_leaf) {

      std::cerr<< "START NormalEstimator"<< std::endl;
      for (int i=0; i<10;i++){
        _scan_with_normals.push_back((Eigen::Vector4f) i);
      }
    // Instantiate a kd_tree on input_cloud
    // for (auto it=src_.begin(); it!=src_.end(); ++it){
    //  std::vector<PointType*> neighbors_of_p;
    //   _kd_tree.fastSearch(neighbors_of_p, p)
    //   if size(neighbors_of_p) < 3:
    //     continue
    //   mean, cov = computeMeanAndCovariance(neighbors_of_p)
    //   eigenvectors_of_cov = getEigenVectors(cov)
    //   normal_of_p = eigenvectors_of_cov.columns(0)
    //   if (-p.dot(normal_of_p) < 0):
    //     normal_of_p = -normal_of_p
    //   point_with_normal = Vector4f();
    //   point_with_normal << p, normal_of_p
    //   _scan_with_normals.push_back(point_with_normal)
    // }
}
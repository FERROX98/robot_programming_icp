#include "normal_estimator.h"

#include "eigen_covariance.h"

//DANIEL, GUIDO
// LaserScanType = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f>>;
NormalEstimator::NormalEstimator(LaserScanType& src_, int max_points_in_leaf)
    : _kd_tree(src_.begin(), src_.end(), max_points_in_leaf) {

    std::cerr<< "START NormalEstimator"<< std::endl;

//  For p in input_cloud:
    for (auto it=src_.begin(); it!=src_.end(); ++it){
      auto& p = *it;
      std::vector<Vector2f*> neighbors_of_p(src_.size());
      std::vector<Vector2f> neighbors_of_p_values(src_.size());
      float norm=0;
      for (size_t i = 0; i < src_.size(); ++i) {
          neighbors_of_p_values[i] = src_[i].head<2>();
          neighbors_of_p[i] = &neighbors_of_p_values[i];
      }
      _kd_tree.fastSearch(neighbors_of_p, p, p.squaredNorm());

    //   if size(neighbors_of_p) < 3:
    //     continue
      if (neighbors_of_p.size() < 3)
            continue;

      Eigen::Matrix<float, 2, 1> mean;
      Eigen::Matrix<float, 2, 2> cov;
      computeMeanAndCovariance(mean,cov, neighbors_of_p_values.begin(),neighbors_of_p_values.end());
  
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
      Eigen::Vector2f eigenvectors_of_cov = es.eigenvectors().col(1);
      Eigen::Vector2f normal_of_p = eigenvectors_of_cov.real();


    //   if (-p.dot(normal_of_p) < 0):
    //     normal_of_p = -normal_of_p
      if (-p.dot(normal_of_p) < 0){
          normal_of_p = -normal_of_p;
      }
    
    //   point_with_normal = Vector4f();
    //   point_with_normal << p, normal_of_p
      Eigen::Vector4f point_with_normal;
      point_with_normal << p, normal_of_p;
        

    //   output_cloud.push_back(point_with_normal)
      _scan_with_normals.push_back(point_with_normal);
    // }
    }
      std::cerr<< "END NormalEstimator"<< std::endl;

}
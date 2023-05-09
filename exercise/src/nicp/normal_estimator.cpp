#include "normal_estimator.h"

#include "eigen_covariance.h"

//DANIEL, GUIDO
NormalEstimator::NormalEstimator(LaserScanType& src_, int max_points_in_leaf)
    : _kd_tree(src_.begin(), src_.end(), max_points_in_leaf) {
      
    _scan_with_normals.clear();

    //std::cerr<< "START NormalEstimator ="<< src_.size()<<std::endl;
    //  For p in input_cloud:
    if ( src_.size()==0){ 
      std::cerr<<"Src vuoto"<< std::endl;
      return;
    }
    std::vector<Vector2f*> neighbors_of_p(src_.size());
    std::vector<Vector2f> neighbors_of_p_values(src_.size());
    
    for (auto it=src_.begin(); it!=src_.end(); ++it){
      
      auto& p = *it;
      // if (neighbors_of_p.empty() )
      //  std::cerr<< "empty"<<std::endl;

    neighbors_of_p.clear();
    neighbors_of_p_values.clear();

      _kd_tree.fastSearch(neighbors_of_p, p, p.norm());

    //   if size(neighbors_of_p) < 3:
    //     continue

      if (neighbors_of_p.size() < 3){
        if(flg_log){
            std::cerr<< "vicini < 3"<<std::endl;
        }
        continue;
      }
      if(flg_log) {
          std::cerr<< "neighbors_of_p.size() >3"<<std::endl;
      }
      for (size_t i = 0; i < neighbors_of_p.size(); ++i) {
          neighbors_of_p_values.push_back( *neighbors_of_p[i]);
         // neighbors_of_p[i] = &neighbors_of_p_values[i];
      }
      Eigen::Matrix<float, 2, 1> mean;
      Eigen::Matrix<float, 2, 2> cov;
      computeMeanAndCovariance(mean,cov, neighbors_of_p_values.begin(),neighbors_of_p_values.end());
  
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es;
        es.compute(cov);
    //  Eigen::Vector2f eigenvectors_of_cov = es.eigenvectors().col(0);
     // Eigen::Vector2f normal_of_p = largestEigenVector(cov);//
       Eigen::Vector2f normal_of_p =es.eigenvectors().col(0);
      //es.eigenvectors().col(0);//eigenvectors_of_cov.real();



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
    }
   // std::cerr<< "END NormalEstimator"<< std::endl;

}
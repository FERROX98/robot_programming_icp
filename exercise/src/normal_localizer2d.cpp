#include "normal_localizer2d.h"
#include <vector>
#include "nicp/eigen_nicp_2d.h"
#include "nicp/normal_estimator.h"


NormalLocalizer2D::NormalLocalizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void NormalLocalizer2D::setMap(std::shared_ptr<Map> map_) {
  
  // Set the internal map pointer
  std::shared_ptr<Map> mappa= map_;
  /*
   * If the map is initialized, fill a temporary vector with world coordinates
   * of all cells representing obstacles.
  */
  // Da salvare direttamente su _obst_vect 
  // std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Vector2f>> temp_vector;
   if ( mappa->initialized() ){
    std::cerr<<"START setMap"<<std::endl;
    int test=0;

    for (int i=0; i<mappa->rows();++i){
      for (int j=0; j<mappa->cols(); ++j){
        if(((CellType) mappa->grid().at(j+i*mappa->cols())) == Occupied){
            if (test){
              std::cerr<< "\n push cell occupied"<<std::endl;
            }
            _obst_vect.push_back(mappa->grid2world(cv::Point2i(i,j)));
        }
        if (test){
          std::cerr <<"\n CellType ="<< (CellType) mappa->grid().at(j+i*mappa->cols()) <<" Grid coord x,y = "<< cv::Point2i(i,j)<<" World Coord -> " <<  mappa->grid2world(cv::Point2i(i,j)) << " END "<<std::endl;
        }
        if (test && j==10)
                break;
        }

      if (test && i==0)
          break;
            
    }
    
  /*
   * If the map is initialized, fill a temporary vector with world coordinates
   * of all cells representing obstacles.
   * Moreover, process normals for these points and store the resulting cloud
   * (with normals) (FIXED SENZA NORMALI) in _obst_vect (Vector2f).
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
  NormalEstimator norm(_obst_vect,20); // scan Vector4f (x,y) (z,v) 

  // Create KD-Tree
  _obst_tree_ptr = std::shared_ptr<TreeType>(new TreeType(_obst_vect.begin(), _obst_vect.end(), 20));
   } 
}
/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */ //GUIDO
void NormalLocalizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  this->_laser_in_world = initial_pose_;

}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 *///DANIEL
void NormalLocalizer2D::process(const ContainerType& scan_) {
  // Use initial pose to get a synthetic scan to compare with scan_
    //TODO
   ContainerType prediction;
    getPrediction(prediction);
    
  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  NICP solver(prediction, scan_,4);
  
  solver.run(200);
  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
  _laser_in_world=_laser_in_world*solver.X();
  
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 *///GUIDO
void NormalLocalizer2D::setLaserParams(float range_min_, float range_max_,
                                        float angle_min_, float angle_max_,
                                        float angle_increment_) {
  this->_range_min = range_min_;
  this->_range_max = range_max_;
  this->_angle_min = angle_min_;
  this->_angle_max = angle_max_;
  this->_angle_increment = angle_increment_;

}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 *///DANIEL
void NormalLocalizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /*
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
  */ 
  
  std::vector<NormalLocalizer2D::LaserPointType*> _obst_vect_search(_obst_vect.size());

      for (size_t i = 0; i < _obst_vect.size(); ++i) {
          _obst_vect_search[i] = &_obst_vect[i];
      }
      // / using LaserContainerType =
     // std::vector<LaserPointType, Eigen::aligned_allocator<LaserPointType>>;

      
  _obst_tree_ptr->fullSearch(_obst_vect_search, _laser_in_world.translation(), _laser_in_world.translation().norm());
  /*
   * You may use additional sensor's informations to refine the prediction.
   * After the prediction is made, augment it with normals using the
   * NormalEstimator
   */

    NormalEstimator norm(_obst_vect,20); // scan Vector4f (x,y) (z,v) 
    norm.get(prediction_);
  }
  
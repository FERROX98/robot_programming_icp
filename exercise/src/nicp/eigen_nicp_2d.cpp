#include "eigen_nicp_2d.h"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <iostream>

#include "rotations.h"

using namespace std;

// TODO correspondeces
NICP::NICP(const ContainerType& fixed_, const ContainerType& moving_,
          int min_points_in_leaf)
    : _fixed_original(fixed_), _fixed(fixed_), _moving(moving_),
    _kd_tree(_fixed.begin(),_fixed.end(),min_points_in_leaf) {}

inline Vector2f skew(const Vector2f& v) { return Vector2f(v(1), -v(0)); }

inline void errorAndJacobian(Eigen::Vector3f& e, Eigen::Matrix3f& J,
                            const PointNormal2f& m, const PointNormal2f& f) {
  const Vector2f pm = m.head<2>();
  const Vector2f nm = m.tail<2>();
  const Vector2f pf = f.head<2>();
  const Vector2f nf = f.tail<2>();
  e(0) = (pm - pf).dot(nm + nf);
  e.block<2, 1>(1, 0) = nm - nf;
  J.block<1, 2>(0, 0) = (nm + nf).transpose();
  J(0, 2) = pm.dot(skew(nf)) + pf.dot(skew(nm));
  J.block<2, 1>(1, 2) = -skew(nm);
}

void NICP::computeCorrespondences() {
  _correspondences.resize(_moving.size());
  int k = 0;
  
  for (const auto& m : _moving) {
    const auto& mt = _X * m;
    auto ft = _kd_tree.bestMatchFast(mt, _ball_radius);
    if (!ft) continue;
    _correspondences[k]._fixed = *ft;
    _correspondences[k]._moving = mt;
    ++k;
  }
  _correspondences.resize(k);
}

void NICP::optimizeCorrespondences() {
  Eigen::Matrix3f H;
  Eigen::Vector3f b;
  H.setZero();
  b.setZero();
  Eigen::Matrix3f J;
  Eigen::Vector3f e;
  J.setZero();
  _num_kernelized = 0;
  _num_inliers = 0;
  _chi2_sum = 0;
  for (const auto& c : _correspondences) {
    const auto& f = c._fixed;
    const auto& m = c._moving;
    errorAndJacobian(e, J, m, f);
    float scale = 1;
    float chi = e.squaredNorm();
    _chi2_sum += chi;
    if (e.squaredNorm() > _kernel_chi2) {
      scale = sqrt(_kernel_chi2 / chi);
      ++_num_kernelized;
    } else {
      ++_num_inliers;
    }
    H.noalias() += scale * J.transpose() * J;
    b.noalias() += scale * J.transpose() * e;
  }
  _dx = H.ldlt().solve(-b);
  Eigen::Isometry2f dX;
  const Eigen::Matrix2f dR = Rtheta(_dx(2));
  dX.setIdentity();
  dX.linear() = dR;
  dX.translation() = _dx.block<2, 1>(0, 0);
  _X = dX * _X;
}

void NICP::run(int max_iterations) {
  int current_iteration = 0;
  while (current_iteration < max_iterations) {
    computeCorrespondences();
    optimizeCorrespondences();
    ++current_iteration;
  }
}

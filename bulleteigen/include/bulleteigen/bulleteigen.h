#ifndef BULLETEIGEN_H
#define BULLETEIGEN_H

#include <Eigen/Eigen>
#include <vector>
#include <memory>

class btConvexShape;

namespace bulleteigen {
  std::shared_ptr<btConvexShape> convertToBulletModel(const std::vector<Eigen::Vector3d>& vertices);
  std::shared_ptr<btConvexShape> convertToBulletModel(const Eigen::MatrixXd& vertices); // [v1, v2, v3 ...]

  bool computeDistance(const std::shared_ptr<btConvexShape>& mesh1,
                       const Eigen::Vector3d& p1,
                       const Eigen::Matrix3d& R1,
                       const std::shared_ptr<btConvexShape>& mesh2,
                       const Eigen::Vector3d& p2,
                       const Eigen::Matrix3d& R2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2);//q1,q2はlocal系. 最近傍点. penetrateしている場合は、penetration depth
}



#endif

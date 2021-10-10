#ifndef VCLIPEIGEN_H
#define VCLIPEIGEN_H

#include <Eigen/Eigen>
#include <vector>
#include <memory>

namespace Vclip{
  class Polyhedron;
}

namespace vclipeigen {
  std::shared_ptr<Vclip::Polyhedron> convertToVClipModel(const std::vector<Eigen::Vector3d>& vertices);

  bool computeDistance(const std::shared_ptr<Vclip::Polyhedron>& mesh1,
                       const Eigen::Vector3d& p1,
                       const Eigen::Matrix3d& R1,
                       const std::shared_ptr<Vclip::Polyhedron>& mesh2,
                       const Eigen::Vector3d& p2,
                       const Eigen::Matrix3d& R2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2);//q1,q2はworld系
}



#endif

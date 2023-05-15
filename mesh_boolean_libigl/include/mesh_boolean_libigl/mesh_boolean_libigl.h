#ifndef MESH_BOOLEAN_LIBIGL_H
#define MESH_BOOLEAN_LIBIGL_H

#include <Eigen/Eigen>

namespace mesh_boolean_libigl{

  // In = [v1 v2 v3 v4 ..]
  // Out = [v1 v2 v3 v4 ..]
  bool intersection(const Eigen::MatrixXd& In1, const Eigen::MatrixXd& In2, Eigen::MatrixXd& Out, bool verbose=false);


};

#endif

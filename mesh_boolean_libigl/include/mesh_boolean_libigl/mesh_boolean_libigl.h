#ifndef MESH_BOOLEAN_LIBIGL_H
#define MESH_BOOLEAN_LIBIGL_H

#include <Eigen/Eigen>

namespace mesh_boolean_libigl{

  // In = [v1 v2 v3 v4 ..]
  // Out = [v1 v2 v3 v4 ..]
  bool intersection(const Eigen::MatrixXd& In1, const Eigen::MatrixXd& In2, Eigen::MatrixXd& Out, bool verbose=false);

  // In = [v1 v2 v3 v4 ..]
  // In = [f1 f2 f3 f4 ..]. f=[idx0, idx1, idx2]^T
  // Out = [v1 v2 v3 v4 ..]
  // Out = [f1 f2 f3 f4 ..]
  bool boolean_union(const Eigen::MatrixXd& InV1, const Eigen::MatrixXi& InF1, const Eigen::MatrixXd& InV2, const Eigen::MatrixXi& InF2, Eigen::MatrixXd& OutV, Eigen::MatrixXi& OutF, bool verbose=false);

  // In = [v1 v2 v3 v4 ..]
  // In = [f1 f2 f3 f4 ..]. f=[idx0, idx1, idx2]^T
  // Out = [v1 v2 v3 v4 ..]
  // Out = [f1 f2 f3 f4 ..]
  bool boolean_intersect(const Eigen::MatrixXd& InV1, const Eigen::MatrixXi& InF1, const Eigen::MatrixXd& InV2, const Eigen::MatrixXi& InF2, Eigen::MatrixXd& OutV, Eigen::MatrixXi& OutF, bool verbose=false);

  // In1 - In2
  // In = [v1 v2 v3 v4 ..]
  // In = [f1 f2 f3 f4 ..]. f=[idx0, idx1, idx2]^T
  // Out = [v1 v2 v3 v4 ..]
  // Out = [f1 f2 f3 f4 ..]
  bool boolean_minus(const Eigen::MatrixXd& InV1, const Eigen::MatrixXi& InF1, const Eigen::MatrixXd& InV2, const Eigen::MatrixXi& InF2, Eigen::MatrixXd& OutV, Eigen::MatrixXi& OutF, bool verbose=false);

  // In = [v1 v2 v3 v4 ..]
  // In = [f1 f2 f3 f4 ..]. f=[idx0, idx1, idx2]^T
  // Out = [v1 v2 v3 v4 ..]
  // Out = [f1 f2 f3 f4 ..]
  bool boolean_xor(const Eigen::MatrixXd& InV1, const Eigen::MatrixXi& InF1, const Eigen::MatrixXd& InV2, const Eigen::MatrixXi& InF2, Eigen::MatrixXd& OutV, Eigen::MatrixXi& OutF, bool verbose=false);

};

#endif

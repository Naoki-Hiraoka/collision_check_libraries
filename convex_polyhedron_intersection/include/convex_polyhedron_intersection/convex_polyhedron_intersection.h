#ifndef CONVEX_POLYHEDRON_INTERSECTION_H
#define CONVEX_POLYHEDRON_INTERSECTION_H

#include <cddeigen/cddeigen.h>

namespace convex_polyhedron_intersection {

  // In = [v1 v2 v3 v4 ..]
  // Out = [v1 v2 v3 v4 ..]
  bool intersection(const Eigen::MatrixXd& In1, const Eigen::MatrixXd& In2, Eigen::MatrixXd& Out, bool verbose=false);

};

#endif


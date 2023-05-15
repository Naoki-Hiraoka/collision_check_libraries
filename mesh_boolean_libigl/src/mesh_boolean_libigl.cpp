#include <mesh_boolean_libigl/mesh_boolean_libigl.h>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <qhulleigen/qhulleigen.h>

namespace mesh_boolean_libigl{

  // In = [v1 v2 v3 v4 ..].
  // Out = [v1 v2 v3 v4 ..]
  bool intersection(const Eigen::MatrixXd& In1, const Eigen::MatrixXd& In2, Eigen::MatrixXd& Out, bool verbose){
    Eigen::MatrixXd V1; // [v1 v2 v3 v4 ..]
    Eigen::MatrixXi F1;
    if(!qhulleigen::convexhull(In1, V1, F1, true)) return false;
    Eigen::MatrixXd V2; // [v1 v2 v3 v4 ..]
    Eigen::MatrixXi F2;
    if(!qhulleigen::convexhull(In2, V2, F2, true)) return false;
    Eigen::MatrixXd V3; // [v1 v2 v3 v4 ..]^T
    Eigen::MatrixXi F3;
    if(!igl::copyleft::cgal::mesh_boolean(V1.transpose(),F1.transpose(),V2.transpose(),F2.transpose(),igl::MESH_BOOLEAN_TYPE_INTERSECT,V3,F3)) return false;
    if(V3.rows() == 0) {
      Out = V3.transpose();
      return true;
    }else{
      return qhulleigen::convexhull(V3.transpose(), Out);
    }
  }

};

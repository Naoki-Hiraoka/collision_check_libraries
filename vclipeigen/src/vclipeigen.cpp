#include <vclipeigen/vclipeigen.h>

#include <vclip.h> // 先にvectorをincludeする必要あり。using namespace stdしてしまうことに注意
#include <iostream>

namespace vclipeigen {
  std::shared_ptr<Vclip::Polyhedron> convertToVClipModel(const std::vector<Eigen::Vector3d>& vertices){
    std::shared_ptr<Vclip::Polyhedron> i_vclip_model = std::make_shared<Vclip::Polyhedron>();
    Vclip::VertFaceName vertName;
    for (int i = 0; i < vertices.size(); i ++ ) {
      sprintf(vertName, "v%d", i);
      i_vclip_model->addVertex(vertName, Vclip::Vect3(vertices[i][0], vertices[i][1], vertices[i][2]));
    }
    i_vclip_model->buildHull();
    i_vclip_model->check();
    return i_vclip_model;
  }

  bool computeDistance(const std::shared_ptr<Vclip::Polyhedron>& mesh1,
                       const Eigen::Vector3d& p1,
                       const Eigen::Matrix3d& R1,
                       const std::shared_ptr<Vclip::Polyhedron>& mesh2,
                       const Eigen::Vector3d& p2,
                       const Eigen::Matrix3d& R2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2//q1,q2はlocal系
                       ){
    if(!mesh1 || mesh2) return false;

    Vclip::Mat3 r1, r2;
    Vclip::Vect3 T1, T2;
    Vclip::VclipPose P1, P2;
    r1.xrow().set(R1(0,0), R1(0,1), R1(0,2));
    r1.yrow().set(R1(1,0), R1(1,1), R1(1,2));
    r1.zrow().set(R1(2,0), R1(2,1), R1(2,2));
    r2.xrow().set(R2(0,0), R2(0,1), R2(0,2));
    r2.yrow().set(R2(1,0), R2(1,1), R2(1,2));
    r2.zrow().set(R2(2,0), R2(2,1), R2(2,2));
    T1.set(p1(0), p1(1), p1(2));
    T2.set(p2(0), p2(1), p2(2));
    P1.set(r1, T1);
    P2.set(r2, T2);
    Vclip::VclipPose X12, X21;
    X12.invert(P2);
    X12.postmult(P1);
    X21.invert(X12);
    Vclip::FeaturePair Feature_Pair;
    // Vclip::VertexはVclip::Featureをprivate継承しているのでshare_ptrだとcastできない
    Feature_Pair.first  = (const Vclip::Feature *)new Vclip::Vertex(mesh1->verts().front());
    Feature_Pair.second = (const Vclip::Feature *)new Vclip::Vertex(mesh2->verts().front());
    Vclip::Vect3 cp1, cp2;
    const Vclip::Polyhedron* Vclip_Model1_raw = mesh1.get();
    const Vclip::Polyhedron* Vclip_Model2_raw = mesh2.get();
    distance = Vclip::Polyhedron::vclip(Vclip_Model1_raw, Vclip_Model2_raw, X12, X21, Feature_Pair.first, Feature_Pair.second, cp1, cp2, 0);
    q1[0] = cp1.x; q1[1] = cp1.y; q1[2] = cp1.z;
    q2[0] = cp2.x; q2[1] = cp2.y; q2[2] = cp2.z;
    return true;
  }
}

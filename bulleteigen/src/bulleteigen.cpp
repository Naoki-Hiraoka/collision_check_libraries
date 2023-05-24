#include <bulleteigen/bulleteigen.h>

#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkCollisionDescription.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btComputeGjkEpaPenetration.h>
#include <LinearMath/btGeometryUtil.h>

#include <iostream>
#include <mutex>

namespace bulleteigen {

  struct ConvexWrap
  {
    std::shared_ptr<btConvexShape> m_convex;
    btTransform m_worldTrans;
    inline btScalar getMargin() const
    {
      return m_convex->getMargin();
    }
    inline btVector3 getObjectCenterInWorld() const
    {
      return m_worldTrans.getOrigin();
    }
    inline const btTransform& getWorldTransform() const
    {
      return m_worldTrans;
    }
    inline btVector3 getLocalSupportWithMargin(const btVector3& dir) const
    {
      return m_convex->localGetSupportingVertex(dir);
    }
    inline btVector3 getLocalSupportWithoutMargin(const btVector3& dir) const
    {
      return m_convex->localGetSupportingVertexWithoutMargin(dir);
    }
  };

  struct btDistanceInfo
  {
    btVector3 m_pointOnA;
    btVector3 m_pointOnB;
    btVector3 m_normalBtoA;
    btScalar m_distance;
  };

  std::shared_ptr<btConvexShape> convertToBulletModel(const std::vector<Eigen::Vector3d>& vertices) {
    std::shared_ptr<btConvexHullShape> i_bullet_model = std::make_shared<btConvexHullShape>();
    //i_bullet_model->setMargin(0.04);
    for (int i = 0; i < vertices.size(); i ++ ) {
      i_bullet_model->addPoint(btVector3(vertices[i][0], vertices[i][1], vertices[i][2]));
    }
    return i_bullet_model;
  }

  bool computeDistance(const std::shared_ptr<btConvexShape>& mesh1,
                       const Eigen::Vector3d& p1,
                       const Eigen::Matrix3d& R1,
                       const std::shared_ptr<btConvexShape>& mesh2,
                       const Eigen::Vector3d& p2,
                       const Eigen::Matrix3d& R2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2//q1,q2はlocal系
                       ){
    if(!mesh1 || !mesh2) return false;

    ConvexWrap a, b;
    a.m_convex = mesh1;
    a.m_worldTrans.setOrigin(btVector3(p1[0], p1[1], p1[2]));
    a.m_worldTrans.setBasis(btMatrix3x3(R1(0,0), R1(0,1), R1(0,2),
                                        R1(1,0), R1(1,1), R1(1,2),
                                        R1(2,0), R1(2,1), R1(2,2)));
    b.m_convex = mesh2;
    b.m_worldTrans.setOrigin(btVector3(p2[0], p2[1], p2[2]));
    b.m_worldTrans.setBasis(btMatrix3x3(R2(0,0), R2(0,1), R2(0,2),
                                        R2(1,0), R2(1,1), R2(1,2),
                                        R2(2,0), R2(2,1), R2(2,2)));

    btGjkCollisionDescription colDesc;
    btVoronoiSimplexSolver simplexSolver;
    btDistanceInfo distInfo;
    int res = -1;
    simplexSolver.reset();
    res = btComputeGjkEpaPenetration(a, b, colDesc, simplexSolver, &distInfo);

    distInfo.m_distance += mesh1->getMargin();
    distInfo.m_pointOnA += mesh1->getMargin() * distInfo.m_normalBtoA;
    distInfo.m_distance += mesh2->getMargin();
    distInfo.m_pointOnB += - mesh2->getMargin() * distInfo.m_normalBtoA;

    distance = distInfo.m_distance;
    Eigen::Vector3d q1_world, q2_world;
    for(int i=0;i<3;i++) q1_world[i] = distInfo.m_pointOnA[i];
    for(int i=0;i<3;i++) q2_world[i] = distInfo.m_pointOnB[i];
    q1 = R1.transpose() * (q1_world - p1);
    q2 = R2.transpose() * (q2_world - p2);

    return true;
  }
}

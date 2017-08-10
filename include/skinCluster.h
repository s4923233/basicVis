#ifndef SKINCLUSTER_H
#define SKINCLUSTER_H

#include "mesh.h"
#include "skeleton.h"

class SkinCluster
{   
    typedef std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > RotationList;

    Mesh* m_mesh;
    Skeleton* m_skeleton;
    Eigen::MatrixXd m_weights;
    std::vector<Eigen::MatrixXd> m_submeshesWeights;

public:
    void bind(Mesh* _mesh, Skeleton* _skeleton);

    void updateMesh(RotationList _rotations, std::vector<Eigen::Vector3d> _absoluteTraslations);
};
#endif


//// Propogate relative rotations via FK to retrieve absolute transformations
//  RotationList vQ;
//  vector<Vector3d> vT;
//  igl::forward_kinematics(C,BE,P,anim_pose,vQ,vT);
//  const int dim = C.cols();
//  MatrixXd T(BE.rows()*(dim+1),dim);
//  for(int e = 0;e<BE.rows();e++)
//  {
//    Affine3d a = Affine3d::Identity();
//    a.translate(vT[e]);
//    a.rotate(vQ[e]);
//    T.block(e*(dim+1),0,dim+1,dim) =
//      a.matrix().transpose().block(0,0,dim+1,dim);
//  }
//  // Compute deformation via LBS as matrix multiplication
//  U = M*T;

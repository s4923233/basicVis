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

public:
    void bind(Mesh* _mesh, Skeleton* _skeleton);

    void updateMesh(RotationList _rotations, std::vector<Eigen::Vector3d> _absoluteTraslations);
};
#endif

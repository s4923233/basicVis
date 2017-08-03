#ifndef MESH_H
#define MESH_H

#include <cstdio>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

class Mesh
{
    friend class SkinCluster;
    Eigen::MatrixXd m_bindPoseVertices;
    Eigen::MatrixXi m_bindPoseFaces;

    Eigen::MatrixXd m_vertices;
    Eigen::MatrixXi m_faces;

public:
    void load(const std::string _filename);

    const Eigen::MatrixXd& vertices()const {return m_vertices;}
    const Eigen::MatrixXi& faces()const {return m_faces;}
};

#endif

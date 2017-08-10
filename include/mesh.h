#ifndef MESH_H
#define MESH_H

#include <cstdio>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "submesh.h"

class Mesh
{
    friend class SkinCluster;
    Eigen::MatrixXd m_bindPoseVertices;
    Eigen::MatrixXi m_bindPoseFaces;

    Eigen::MatrixXd m_vertices;
    Eigen::MatrixXi m_faces;
    std::vector<SubMesh> m_submeshes;

public:
    Mesh();

    void load(const std::string _filename);
    void loadSubMesh(const std::string _filename);

    const Eigen::MatrixXd& vertices()const {return m_vertices;}
    const Eigen::MatrixXi& faces()const {return m_faces;}
    const SubMesh& submesh(size_t _index)const {return m_submeshes[_index];}

private:
    //TODO
    void grid(const Eigen::MatrixXd& _mesh, Eigen::MatrixXd& _grid, Eigen::Matrix<double,8,3>& _bbox, Eigen::Matrix<int,12,2>& _e_box);
    void hrbf(const Eigen::MatrixXd& _vertices,const Eigen::MatrixXd& _normals,const Eigen::MatrixXd& _grid,Eigen::VectorXd& _field,Eigen::MatrixXd& _grad);
    void slice(const Eigen::MatrixXd& _grid, const Eigen::MatrixXd& _hrbf, Eigen::MatrixXd& _slicePlane,Eigen::MatrixXi& _slicePlaneFaces,Eigen::MatrixXd& _colour);

};

#endif

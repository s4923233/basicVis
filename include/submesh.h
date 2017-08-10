#ifndef SUBMESH_H
#define SUBMESH_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>
class SubMesh
{
    Eigen::MatrixXd m_poissons;
    Eigen::MatrixXd m_normals;

    Eigen::MatrixXd m_grid;
    Eigen::Matrix<double,8,3> m_bbox;
    Eigen::Matrix<int,12,2> m_boxEdges;

    Eigen::VectorXd m_hrbf;//hrbf field
    Eigen::MatrixXd m_grad;//hrbf gradient

    Eigen::MatrixXd m_slicePlaneVertices;
    Eigen::MatrixXi m_slicePlaneFaces;
    Eigen::MatrixXd m_slicePlaneColour;

public:
    const Eigen::MatrixXd& vertices()const                              {return m_poissons;}
    const Eigen::MatrixXd& normals()const                               {return m_normals;}

    const Eigen::MatrixXd& gridVertices()const                          {return m_grid;}
    const Eigen::Matrix<double,8,3>& bbox()const                        {return m_bbox;}
    const Eigen::Matrix<int,12,2>& bboxEdges()const                     {return m_boxEdges;}

    const Eigen::VectorXd& field()const                                 {return m_hrbf;}
    const Eigen::MatrixXd& grad()const                                  {return m_grad;}

    const Eigen::MatrixXd& slicePlaneVertices()const                    {return m_slicePlaneVertices;}
    const Eigen::MatrixXi& slicePlaneFaces()const                       {return m_slicePlaneFaces;}
    const Eigen::MatrixXd& slicePlaneColour()const                      {return m_slicePlaneColour;}

    SubMesh();
    SubMesh(const Eigen::MatrixXd &_poissons, const Eigen::MatrixXd &_normals);
//    SubMesh(const SubMesh& _other);
//    SubMesh& operator=(const SubMesh& _other);
//    ~SubMesh();

    void setGridVertices(const Eigen::MatrixXd& _gridVertices);
    void setBboxVerices(const Eigen::MatrixXd& _bboxVertices);
    void setSlicePlaneVertices(const Eigen::MatrixXd& _slicePlaneVertices);

private:
    void grid(const Eigen::MatrixXd& _mesh, Eigen::MatrixXd& _grid, Eigen::Matrix<double,8,3>& _bbox, Eigen::Matrix<int,12,2>& _e_box);
    void hrbf(const Eigen::MatrixXd& _vertices,const Eigen::MatrixXd& _normals,const Eigen::MatrixXd& _grid,Eigen::VectorXd& _field,Eigen::MatrixXd& _grad);
    void slice(const Eigen::MatrixXd& _grid, const Eigen::MatrixXd& _hrbf, Eigen::MatrixXd& _slicePlane, Eigen::MatrixXi &_slicePlaneFaces, Eigen::MatrixXd& _colour);

};
#endif

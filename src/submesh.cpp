#include "submesh.h"

#include <igl/jet.h>

#include "hrbf_phi_funcs.h"
#include "hrbf_core.h"
#include <vector>

SubMesh::SubMesh()
{

}

SubMesh::SubMesh(const Eigen::MatrixXd& _poissons,const Eigen::MatrixXd& _normals)
{
    m_poissons = _poissons;
    m_normals = _normals;

    grid(m_poissons,m_grid,m_bbox,m_boxEdges);
    hrbf(m_poissons,m_normals,m_grid,m_hrbf,m_grad);
    slice(m_grid,m_hrbf,m_slicePlaneVertices,m_slicePlaneFaces,m_slicePlaneColour);
}
/*
SubMesh::SubMesh(const SubMesh& _other)
{
    this->m_poissons = _other.m_poissons;
    this->m_normals= _other.m_normals;

    this->m_grid= _other.m_grid;
    this->m_bbox= _other.m_bbox;
    this->m_boxEdges= _other.m_boxEdges;

    this->m_hrbf= _other.m_hrbf;
    this->m_grad= _other.m_grad;

    this->m_slicePlaneVertices= _other.m_slicePlaneVertices;
    this->m_slicePlaneFaces= _other.m_slicePlaneFaces;
    this->m_slicePlaneColour= _other.m_slicePlaneColour;

}

SubMesh& SubMesh::operator=(const SubMesh& _other)
{
    assert(this != &_other);
    this->m_poissons = _other.m_poissons;
    this->m_normals= _other.m_normals;

    this->m_grid= _other.m_grid;
    this->m_bbox= _other.m_bbox;
    this->m_boxEdges= _other.m_boxEdges;

    this->m_hrbf= _other.m_hrbf;
    this->m_grad= _other.m_grad;

    this->m_slicePlaneVertices= _other.m_slicePlaneVertices;
    this->m_slicePlaneFaces= _other.m_slicePlaneFaces;
    this->m_slicePlaneColour= _other.m_slicePlaneColour;

    return *this;
}

SubMesh::~SubMesh(){}
*/

void SubMesh::setGridVertices(const Eigen::MatrixXd& _gridVertices)
{
    m_grid = _gridVertices;
}

void SubMesh::setBboxVerices(const Eigen::MatrixXd& _bboxVertices)
{
    m_bbox = _bboxVertices;
}

void SubMesh::setSlicePlaneVertices(const Eigen::MatrixXd& _slicePlaneVertices)
{
    m_slicePlaneVertices = _slicePlaneVertices;
}


void SubMesh::grid(const Eigen::MatrixXd& _mesh, Eigen::MatrixXd& _grid, Eigen::Matrix<double,8,3>& _bbox, Eigen::Matrix<int,12,2>& _boxEdges)
{

    size_t resolution = 32;//2^3
    _grid = Eigen::MatrixXd(resolution*resolution*resolution,3);//8^3


    Eigen::Vector3d min = _mesh.colwise().minCoeff();
    Eigen::Vector3d max = _mesh.colwise().maxCoeff();
    Eigen::Vector3d delta = (max-min)/(resolution-1);

    //expand the bbox to prevent cuts in isosurface
    min = min - delta*0.5;
    max = max + delta*0.5;

    //bbox
    _bbox<<
    min(0), min(1), min(2),
    max(0), min(1), min(2),
    max(0), max(1), min(2),
    min(0), max(1), min(2),
    min(0), min(1), max(2),
    max(0), min(1), max(2),
    max(0), max(1), max(2),
    min(0), max(1), max(2);

    //bbox edges
    _boxEdges <<
            0, 1,
            1, 2,
            2, 3,
            3, 0,
            4, 5,
            5, 6,
            6, 7,
            7, 4,
            0, 4,
            1, 5,
            2, 6,
            7 ,3;


    delta = (max-min)/(resolution-1);

    for(size_t z = 0; z < resolution; z++){
        for(size_t y = 0; y < resolution; y++){
            for(size_t x = 0; x < resolution; x++){
                _grid.row(x+resolution*(y+resolution*z))=Eigen::RowVector3d(min(0)+delta(0)*x,
                                                                            min(1)+delta(1)*y,
                                                                            min(2)+delta(2)*z);
            }
        }
    }
}

void SubMesh::hrbf(const Eigen::MatrixXd& _vertices,const Eigen::MatrixXd& _normals,const Eigen::MatrixXd& _grid,Eigen::VectorXd& _field,Eigen::MatrixXd& _grad)
{
    typedef HRBF_fit<float, 3, Rbf_pow3<float> > HRBF;

    //Hermite rbf
    HRBF fit;
    std::vector<HRBF::Vector> v_pts;
    std::vector<HRBF::Vector> v_nrs;
    for(int i = 0; i<_vertices.rows();++i)
    {
        v_pts.push_back(HRBF::Vector(_vertices(i,0),_vertices(i,1),_vertices(i,2)));
        v_nrs.push_back(HRBF::Vector(_normals(i,0),_normals(i,1),_normals(i,2)));
    }

    fit.hermite_fit(v_pts, v_nrs);

    _field = Eigen::VectorXd::Zero(_grid.rows());
    //_grad = Eigen::MatrixXd(_grid.rows(),3);
    for(size_t indx=0; indx<_grid.rows();indx++)
    {
        Eigen::RowVector3d p = _grid.row(indx);
        HRBF::Vector x(p(0),p(1),p(2));

        _field(indx) = fit.eval(x);
        //_grad.row(indx) = Eigen::RowVector3d(fit.grad(x));

    }

    //Re-parametrization
    double maxDist = _vertices.colwise().maxCoeff().norm();

    Eigen::VectorXd T = Eigen::VectorXd::Zero(_grid.rows());
    for(size_t indx = 0; indx<_grid.rows(); indx++)
    {

        double x = _field(indx);//d_i(X)

        //if field(index)
        if(x < -maxDist)
        {
            T(indx) = 1.0;
            continue;
        }
        if(x > maxDist)
        {
            T(indx) = 0.0;
            continue;
        }

        //T(p) = -3/16*(x/maxDist)^5 + 5/8*(x/maxDist)^3 - 15/16(x/maxDist)+1/2;
        double x_r = x/maxDist;
        T(indx) = (-3.0/16.0)*pow(x_r,5)+
                (5.0/8.0)*pow(x_r,3)-
                (15.0/16.0)*x_r + 0.5;

    }

    //_field = T;
}

void SubMesh::slice(const Eigen::MatrixXd& _grid, const Eigen::MatrixXd& _hrbf, Eigen::MatrixXd& _slicePlane,Eigen::MatrixXi& _slicePlaneFaces,Eigen::MatrixXd& _colour)
{

    //2nd method
    size_t resX = 32;
    size_t resY = 32;
    size_t z_slice = 1;

    size_t begin = resX*resY*z_slice;
    size_t end = resX*resY*(z_slice+1);

    _slicePlane = Eigen::MatrixXd(resX*resY,3);
    Eigen::VectorXd dValue = Eigen::VectorXd::Zero(resX*resY);

    int indx = 0;
    for(size_t i = begin; i<end;++i)
    {
        _slicePlane.row(indx) = _grid.row(i);
        dValue(indx++) = _hrbf(i);
    }

    //set faces
    _slicePlaneFaces = Eigen::MatrixXi((resX-1)*(resY-1)*2,3);
    size_t i = 0;
    size_t le_index; //lower edge index
    size_t ue_index; //upper edge index
    for(size_t y = 0; y<resY-1; y++)
    {
        for(size_t x = 0; x<resX-1;x++)
        {
            le_index = y*(resX)+x;
            ue_index = (y+1)*(resY)+x;
            _slicePlaneFaces.row(i++) = Eigen::RowVector3i(le_index,le_index+1,ue_index);
            _slicePlaneFaces.row(i++) = Eigen::RowVector3i(le_index+1,ue_index+1,ue_index);

        }
    }


    igl::jet(dValue,true,_colour);
}

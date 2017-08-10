#include "mesh.h"

#include <igl/readOBJ.h>

#include <igl/jet.h>

#include "hrbf_phi_funcs.h"
#include "hrbf_core.h"
#include <vector>

Mesh::Mesh()
{
//    foreach submesh concat
//    MatrixXd D(A.rows()+B.rows(), A.cols());
//    D << A,
//         B;

//    grid(m_poissons,m_grid,m_bbox,m_boxEdges);
//    hrbf(m_poissons,m_normals,m_grid,m_hrbf,m_grad);
//    slice(m_grid,m_hrbf,m_slicePlaneVertices,m_slicePlaneFaces,m_slicePlaneColour);
}

void Mesh::load(const std::string _filename)
{
    igl::readOBJ(_filename,m_vertices,m_faces);
}

void Mesh::loadSubMesh(const std::string _filename)
{
    std::vector<std::vector<double > > temp_V;
    std::vector<std::vector<double > > temp_N;
    std::vector<std::vector<double > > temp_TC;

    std::vector<std::vector<int > > temp_F;
    std::vector<std::vector<int > > temp_FTC;
    std::vector<std::vector<int > > temp_FN;
    igl::readOBJ(_filename,
                 temp_V,
                 temp_TC,
                 temp_N,
                 temp_F,
                 temp_FTC,
                 temp_FN);

//REMOVE THIS BIT ONCE THE INPUT MESHES ARE FIXED ----------------------------------
    //convert V,N in eigen::MatrixXd
    int reductionRate = temp_V.size()/50;//50 is the target size
    //delete points are too many
    for(int i= 1; i<temp_V.size()-reductionRate-1;++i)
    {
        temp_V.erase(temp_V.begin()+i,temp_V.begin()+reductionRate+i);
        temp_N.erase(temp_N.begin()+i,temp_N.begin()+reductionRate+i);
        //pts.push_back(v_pts[i*20]);
    }
////--------------------------------------------------------------------------------

    Eigen::MatrixXd V(temp_V.size(), 3);
    for (int i = 0; i < temp_V.size(); i++)
        V.row(i) = Eigen::Map<Eigen::RowVectorXd>(&temp_V[i][0],temp_V[i].size());

    Eigen::MatrixXd N(temp_N.size(), 3);
    for (int i = 0; i < temp_N.size(); i++)
        N.row(i) = Eigen::Map<Eigen::RowVectorXd>(&temp_N[i][0],temp_N[i].size());

    m_submeshes.push_back(SubMesh(V,N));
}


void Mesh::grid(const Eigen::MatrixXd& _mesh, Eigen::MatrixXd& _grid, Eigen::Matrix<double,8,3>& _bbox, Eigen::Matrix<int,12,2>& _boxEdges)
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

void Mesh::hrbf(const Eigen::MatrixXd& _vertices,const Eigen::MatrixXd& _normals,const Eigen::MatrixXd& _grid,Eigen::VectorXd& _field,Eigen::MatrixXd& _grad)
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

void Mesh::slice(const Eigen::MatrixXd& _grid, const Eigen::MatrixXd& _hrbf, Eigen::MatrixXd& _slicePlane,Eigen::MatrixXi& _slicePlaneFaces,Eigen::MatrixXd& _colour)
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

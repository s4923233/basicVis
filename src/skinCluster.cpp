#include "skinCluster.h"
#include "submesh.h"
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/dqs.h>
#include <igl/boundary_conditions.h>
#include <igl/bbw.h>

void SkinCluster::bind(Mesh* _mesh, Skeleton* _skeleton)
{

    m_mesh = _mesh;
    m_skeleton = _skeleton;

    Eigen::MatrixXd temp_vertices;
    Eigen::MatrixXi temp_faces, tetrahedrons;

    igl::copyleft::tetgen::tetrahedralize(m_mesh->vertices(),
                                          m_mesh->faces(),
                                          "pq1.414Y",
                                          temp_vertices,
                                          tetrahedrons,
                                          temp_faces);

    m_mesh->m_bindPoseVertices = temp_vertices;
    m_mesh->m_bindPoseFaces = temp_faces;

    m_mesh->m_vertices = temp_vertices;
    m_mesh->m_faces = temp_faces;


    Eigen::VectorXi boundaries;
    // List of boundary conditions of each weight function
    Eigen::MatrixXd boundaryConditions;

    igl::boundary_conditions(m_mesh->vertices(),
                             tetrahedrons,
                             m_skeleton->joints(),
                             Eigen::VectorXi(),
                             m_skeleton->boneEdges(),
                             Eigen::MatrixXi(),
                             boundaries,
                             boundaryConditions);

    // compute Bounded Biharmonic Weights matrix
    igl::BBWData bbw_data;

    bbw_data.active_set_params.max_iter = 8;
    bbw_data.verbosity = 2;


    if(!igl::bbw(m_mesh->vertices(),
                 tetrahedrons,
                 boundaries,
                 boundaryConditions,
                 bbw_data,
                 m_weights))
    {
      return;
    }

    //Normalise weights to sum to one
    igl::normalize_row_sums(m_weights,m_weights);

}

//skeleton notify->skinCluster
//skinCluster.onNotify(): updateMesh()
void SkinCluster::updateMesh(RotationList _absoluteRotations, std::vector<Eigen::Vector3d> _absoluteTraslations)
{
    igl::dqs(m_mesh->m_bindPoseVertices,
             m_weights,
             _absoluteRotations,
             _absoluteTraslations,
             m_mesh->m_vertices);


    size_t nBoneEdges = m_skeleton->boneEdges().rows();

    Eigen::MatrixXd G,P,BBox;
    for(int bone_i=0; bone_i<nBoneEdges;bone_i++)
    {

        Eigen::Affine3d A = Eigen::Affine3d::Identity();
        A.translate(_absoluteTraslations[bone_i]);
        A.rotate(_absoluteRotations[bone_i]);

//        Eigen::Affine3d Translate = Eigen::Affine3d::Identity();
//        Translate.translate(_absoluteTraslations[bone_i]);

//        Eigen::Affine3d Rotate = Eigen::Affine3d::Identity();
//        Rotate.rotate(_absoluteRotations[bone_i]);

//        Eigen::Affine3d A = Eigen::Affine3d::Identity();
//        A = Rotate*Translate;

        SubMesh& submesh = m_mesh->m_submeshes[bone_i];

        //grid vertices
        G = submesh.gridVertices();
        G.transpose() = A.linear()*G.transpose();
        submesh.setGridVertices(G);

        //slice plane
        P= submesh.slicePlaneVertices();
        P.transpose() = A.linear()*P.transpose();
        submesh.setSlicePlaneVertices(P);

        //bounding box
        BBox= submesh.bbox();
        BBox.transpose() = A.linear()*BBox.transpose();
        submesh.setBboxVerices(BBox);

    }


    //update the mesh's grid
    //grid(m_vertices) or grid(submesh.gridpoints) <-this one is safer

}

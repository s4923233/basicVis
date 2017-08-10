#include <igl/viewer/Viewer.h>
#include "skeleton.h"
#include "mesh.h"
#include "skinCluster.h"
#include "utils.h"

#include <igl/forward_kinematics.h>
#include <igl/deform_skeleton.h>
#include <igl/copyleft/marching_cubes.h>
#include <igl/readOBJ.h>

#ifndef FILEPATH
#define FILEPATH "/Users/federicoleone/Documents/maya/projects/Master/data"
#endif

#ifndef TUTORIAL_SHARED_PATH
#define TUTORIAL_SHARED_PATH "/Users/federicoleone/libigl/tutorial/shared"
#endif

typedef std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > RotationList;

RotationList pose;
double anim_t = 1.0;
double anim_t_dir = -0.03;
Skeleton skeleton;
Mesh mesh;
SkinCluster skinCluster;

bool pre_draw(igl::viewer::Viewer & _viewer)
{
    if(_viewer.core.is_animating)
    {
    RotationList anim_pose(pose.size());
    for(int i = 0;i<pose.size();i++)
    {
      anim_pose[i] = pose[i].slerp(anim_t,Eigen::Quaterniond::Identity());
    }

    skeleton.pose(anim_pose);

   // alpha = alpha< 1.0f ? alpha + 0.01 : 0.0f;
                            //Check world space transformations
    skinCluster.updateMesh(skeleton.worldSpaceRotations(),skeleton.worldSpaceTraslations());

    _viewer.data.set_points(skeleton.joints(),Eigen::RowVector3d(1.0,0.0,0.0));
    //_viewer.data.set_edges(skeleton.joints(),skeleton.boneEdges(),Eigen::RowVector3d(1.0,1.0,0.0));
    _viewer.data.set_edges(mesh.submesh(1).bbox(),mesh.submesh(1).bboxEdges(),Eigen::RowVector3d(1.0,1.0,0.0));
//    _viewer.data.set_points(mesh.submesh(1).gridVertices(),Eigen::RowVector3d(0.75,0.75,0.0));

//    _viewer.data.set_mesh(mesh.vertices(),mesh.faces());

    //_viewer.data.set_mesh(mesh.submesh(1).slicePlaneVertices(),mesh.submesh(1).slicePlaneFaces());
    //_viewer.data.set_colors(mesh.submesh(1).slicePlaneColour());

    anim_t += anim_t_dir;
    anim_t_dir *= (anim_t>=1.0 || anim_t<=0.0?-1.0:1.0);
}
    return false;
}

bool key_down(igl::viewer::Viewer &viewer, unsigned char key, int mods)
{
  switch(key)
  {
    case ' ':
      viewer.core.is_animating = !viewer.core.is_animating;
      break;
  }
  return true;
}


int main(int argc, char *argv[])
{

    mesh.load(FILEPATH "/capsuleTrimesh.obj");

    mesh.loadSubMesh(FILEPATH "/poissonSubMeshA.obj");
    mesh.loadSubMesh(FILEPATH "/poissonSubMeshB.obj");
    mesh.loadSubMesh(FILEPATH "/poissonSubMeshC.obj");

//  Skeleton skeleton;
    skeleton.load(FILEPATH "/capsule_threeJnt_setup.tgf");

   skinCluster.bind(&mesh,&skeleton);

    //building a rotationList explicitly
    {
        Eigen::Quaterniond q(igl::PI,0,1,0);
        q.normalize();
        pose = skeleton.jointOrientation();
        pose[1] = pose[1]*q*pose[1].inverse();
        q.w() = igl::PI/2.0f;
        q.vec() = Eigen::Vector3d(0,-1,0);
        pose[2] = pose[2]*q*pose[2].inverse();
    }//RotationList end

    Eigen::MatrixXd gridpoints;
    Eigen::Matrix<double,8,3> bbox;
    Eigen::Matrix<int,12,2> e_box;

    grid(mesh.vertices(),gridpoints,bbox,e_box);


//    Eigen::VectorXd field;
//    Eigen::MatrixXd grad;

    Eigen::MatrixXd surfaceVertices;
    Eigen::MatrixXi surfaceFaces;
    igl::copyleft::marching_cubes(mesh.submesh(1).field(),mesh.submesh(1).gridVertices(),32,32,32,surfaceVertices,surfaceFaces);
/*

    Eigen::MatrixXd slicePlane;
    Eigen::MatrixXi slicePlaneFaces;
    Eigen::MatrixXd colour;
    slice(gridpoints, field,slicePlane,slicePlaneFaces,colour);
*/

    igl::viewer::Viewer viewer;

    //show all the grids
    // Create one huge mesh containing both meshes
    //igl::cat(1,low.U,high.U,scene.U);
    //igl::cat(1,low.F,MatrixXi(high.F.array()+low.V.rows()),scene.F);
    // Color each mesh
    //viewer.data.set_mesh(scene.U,scene.F);


   //viewer.data.set_mesh(mesh.vertices(),mesh.faces());
   viewer.data.set_points(skeleton.joints(),Eigen::RowVector3d(1.0,0.0,0.0));

   viewer.data.set_edges(skeleton.joints(),skeleton.boneEdges(),Eigen::RowVector3d(1.0,1.0,0.0));

   //viewer.data.set_points(mesh.submesh(1).gridVertices(),Eigen::RowVector3d(0.75,0.75,0.0));
    viewer.data.set_edges(mesh.submesh(1).bbox(),mesh.submesh(1).bboxEdges(),Eigen::RowVector3d(1.0,1.0,0.0));

    viewer.data.set_mesh(surfaceVertices,surfaceFaces);

    //viewer.data.set_mesh(mesh.submesh(1).slicePlaneVertices(),mesh.submesh(1).slicePlaneFaces());
   //viewer.data.set_colors(mesh.submesh(1).slicePlaneColour());


    viewer.callback_pre_draw = &pre_draw;
    viewer.callback_key_down = &key_down;
    viewer.core.is_animating = false;
    viewer.core.animation_max_fps = 30.;

    viewer.launch();
}

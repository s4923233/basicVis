#include <igl/viewer/Viewer.h>
#include "skeleton.h"
#include "mesh.h"
#include "skinCluster.h"

#include <igl/forward_kinematics.h>
#include <igl/deform_skeleton.h>

#ifndef FILEPATH
#define FILEPATH "/Users/federicoleone/Documents/maya/projects/Master/data"
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
    skinCluster.updateMesh(skeleton.worldSpaceRotations(),skeleton.worldSpaceTraslations());

    _viewer.data.set_points(skeleton.joints(),Eigen::RowVector3d(1.0,0.0,0.0));
    _viewer.data.set_edges(skeleton.joints(),skeleton.boneEdges(),Eigen::RowVector3d(1.0,1.0,0.0));

    _viewer.data.set_mesh(mesh.vertices(),mesh.faces());

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


    igl::viewer::Viewer viewer;

    viewer.data.set_mesh(mesh.vertices(),mesh.faces());
    viewer.data.set_points(skeleton.joints(),Eigen::RowVector3d(1.0,0.0,0.0));

    viewer.data.set_edges(skeleton.joints(),skeleton.boneEdges(),Eigen::RowVector3d(1.0,1.0,0.0));
    viewer.callback_pre_draw = &pre_draw;
    viewer.callback_key_down = &key_down;
    viewer.core.is_animating = false;
    viewer.core.animation_max_fps = 30.;

    viewer.launch();
}

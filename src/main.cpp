#include <igl/viewer/Viewer.h>
#include "skeleton.h"
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

    _viewer.data.set_points(skeleton.joints(),Eigen::RowVector3d(1.0,0.0,0.0));
    _viewer.data.set_edges(skeleton.joints(),skeleton.boneEdges(),Eigen::RowVector3d(1.0,1.0,0.0));


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
//    Skeleton skeleton;
    skeleton.load(FILEPATH "/capsule_threeJnt_setup.tgf");

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
//    skeleton.pose(jointOrientation);

    {
        //skeleton.setLabel("JointA_1",1);
        //skeleton.setLabel("JointB_1",2);
//        skeleton.setLabel(FILEPATH "/capsule_threeJnt_labels.tgf");
//        Eigen::Quaterniond q(igl::PI,0,0,1);
//        q.w() *= 0.25;
//        skeleton.pose("JointB_1",q);
//        skeleton.pose("JointC_1",Eigen::Quaterniond(igl::PI/2,0,0,-1));
//        poses.resize(2,RotationList(3,Eigen::Quaterniond::Identity()));
//        poses[1] = skeleton.jointOrientation();
    }

    //skeleton.reset();
    // Plot the mesh
    igl::viewer::Viewer viewer;
    viewer.data.set_points(skeleton.joints(),Eigen::RowVector3d(1.0,0.0,0.0));

    viewer.data.set_edges(skeleton.joints(),skeleton.boneEdges(),Eigen::RowVector3d(1.0,1.0,0.0));
    viewer.callback_pre_draw = &pre_draw;
    viewer.callback_key_down = &key_down;
    viewer.core.is_animating = false;
    viewer.core.animation_max_fps = 30.;

    viewer.launch();
}


//Quaternion:
//  Eigen::Quaterniond q(2, 0, 1, -3);
//  std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " << std::endl << q.vec() << std::endl;

//  q.normalize();
//  std::cout << "To represent rotation, we need to normalize it such that its length is " << q.norm() << std::endl;

//  Eigen::Vector3d v(1, 2, -1);
//  Eigen::Quaterniond p;
//  p.w() = 0;
//  p.vec() = v;
//  Eigen::Quaterniond rotatedP = q * p * q.inverse();
//  Eigen::Vector3d rotatedV = rotatedP.vec();
//  std::cout << "We can now use it to rotate a vector " << std::endl << v << " to " << std::endl << rotatedV << std::endl;

//  Eigen::Matrix3d R = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
//  std::cout << "Compare with the result using an rotation matrix " << std::endl << R * v << std::endl;

//  Eigen::Quaterniond a = Eigen::Quterniond::Identity();
//  Eigen::Quaterniond b = Eigen::Quterniond::Identity();
//  Eigen::Quaterniond c; // Adding two quaternion as two 4x1 vectors is not supported by the EIgen API. That is, c = a + b is not allowed. We have to do this in a hard way
//  c.w() = a.w() + b.w();
//  c.x() = a.x() + b.x();
//  c.y() = a.y() + b.y();
//  c.z() = a.z() + b.z();

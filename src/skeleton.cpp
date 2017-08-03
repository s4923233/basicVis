#include "skeleton.h"
#include "igl/PI.h"

void Skeleton::load(const std::string _tgf_filename)
{
    igl::readTGF(_tgf_filename,m_BindPosejoints,m_BindPoseBoneEdges);
    igl::directed_edge_parents(m_BindPoseBoneEdges,m_parent);
    m_joints = m_BindPosejoints;
    m_boneEdges = m_BindPoseBoneEdges;
    m_jointOrientation = RotationList(m_boneEdges.rows(),Eigen::Quaterniond::Identity());
}

//improve this method.
void Skeleton::pose(RotationList& _pose)
{
    igl::forward_kinematics(m_BindPosejoints,m_BindPoseBoneEdges,m_parent,_pose,m_worldSpaceRotations,m_worldSpaceTraslations);

    const int dim = m_BindPosejoints.cols();
    m_transformationMatrix = Eigen::MatrixXd(m_BindPoseBoneEdges.rows()*(dim+1),dim);
    for(size_t bone_index = 0;bone_index<m_BindPoseBoneEdges.rows();bone_index++)
    {
      Eigen::Affine3d a = Eigen::Affine3d::Identity();
      a.translate(m_worldSpaceTraslations[bone_index]);
      a.rotate(m_worldSpaceRotations[bone_index]);
      m_transformationMatrix.block(bone_index*(dim+1),0,dim+1,dim) = a.matrix().transpose().block(0,0,dim+1,dim);
    }


    igl::deform_skeleton(m_BindPosejoints,
                         m_BindPoseBoneEdges,
                         m_transformationMatrix,
                         m_joints,
                         m_boneEdges);

}

void Skeleton::pose(const std::string _label, Eigen::Quaterniond _q)
{
    size_t jointIndex = m_jointsLabel.at(_label);
    _q.normalize();

    RotationList jointOrientation = m_jointOrientation;
    jointOrientation[jointIndex] = m_jointOrientation[jointIndex]*_q*m_jointOrientation[jointIndex].inverse();

    pose(jointOrientation);
}

//TODO
void Skeleton::reset()
{
//load the stored initial value and pose the skeleton
}

void Skeleton::reset(const std::string _label)
{
//load the stored initial value for a specific joint and pose the skeleton
}


void Skeleton::setLabel(const std::string _label,size_t _jointIndex)
{
    m_jointsLabel[_label] = _jointIndex;
}

//TODO
//Improve the method to read a tgf file with comments, indices, labels
void Skeleton::setLabel(const std::string _filename)
{

    std::ifstream file;
    file.open(_filename);
    if(!file.is_open())
    {
        std::cout<<"IOError: "<<_filename.c_str()<< " cannot be found\n";
        return;
    }

    std::string line;
    size_t index = 1;
    std::string label;
    while(std::getline(file,line))
    {
        //TODO extract label and index from line
        //label = getLabel(line);
        //index = getIndex(line);
        label = line;
        assert(index-1<m_joints.rows());

        if(label.empty())
        {
            index++;
            continue;
        }

        m_jointsLabel[label] = index-1;
        index++;
    }
    file.close();

    for(auto elem : m_jointsLabel)
    {
       std::cout << elem.first << " " << elem.second<< "\n";
    }
}

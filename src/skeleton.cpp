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
    RotationList vQ;
    std::vector<Eigen::Vector3d> vT;
    igl::forward_kinematics(m_BindPosejoints,m_BindPoseBoneEdges,m_parent,_pose,vQ,vT);

    const int dim = m_BindPosejoints.cols();
    Eigen::MatrixXd T(m_BindPoseBoneEdges.rows()*(dim+1),dim);
    for(int e = 0;e<m_BindPoseBoneEdges.rows();e++)
    {
      Eigen::Affine3d a = Eigen::Affine3d::Identity();
      a.translate(vT[e]);
      a.rotate(vQ[e]);
      T.block(e*(dim+1),0,dim+1,dim) = a.matrix().transpose().block(0,0,dim+1,dim);
    }

    Eigen::MatrixXd CT;
    Eigen::MatrixXi BET;
    igl::deform_skeleton(m_BindPosejoints,m_BindPoseBoneEdges,T,CT,BET);

    m_joints = CT;
    m_boneEdges = BET;
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
//load the stored initial value for a specific koint and pose the skeleton
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

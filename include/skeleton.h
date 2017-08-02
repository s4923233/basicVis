#include <cstdio>
#include <vector>

#include <igl/directed_edge_parents.h>
#include <igl/readTGF.h>
#include <igl/forward_kinematics.h>
#include <igl/deform_skeleton.h>


#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <map>

class Skeleton
{
    typedef std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > RotationList;
    Eigen::MatrixXd m_BindPosejoints;
    Eigen::MatrixXi m_BindPoseBoneEdges;

    Eigen::MatrixXd m_joints;
    Eigen::MatrixXi m_boneEdges;
    Eigen::VectorXi m_parent;
    RotationList m_jointOrientation;
    std::map<std::string,size_t> m_jointsLabel;

    public:
    void load(const std::string _tgf_filename);

    const Eigen::MatrixXd& joints()const                {return m_joints;}
    const Eigen::MatrixXi& boneEdges()const             {return m_boneEdges;}
    const Eigen::VectorXi& boneParent()const            {return m_parent;}
    const RotationList& jointOrientation()const         {return m_jointOrientation;}

    void pose(RotationList& _pose);
    void pose(const std::string _label, Eigen::Quaterniond _q);

    void reset();
    void reset(const std::string _label);

    void setLabel(const std::string _label,size_t _jointIndex);
    void setLabel(const std::string _filename);
};

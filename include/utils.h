#include <cstdio>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

void grid(const Eigen::MatrixXd& _mesh, Eigen::MatrixXd& _grid, Eigen::Matrix<double,8,3>& _bbox, Eigen::Matrix<int,12,2>& _e_box);
void hrbf(const Eigen::MatrixXd& _vertices,const Eigen::MatrixXd& _normals,const Eigen::MatrixXd& _grid,Eigen::VectorXd& _field,Eigen::MatrixXd& _grad);
void slice(const Eigen::MatrixXd& _grid, const Eigen::MatrixXd& _hrbf, Eigen::MatrixXd& _slicePlane, Eigen::MatrixXi &_slicePlaneFaces, Eigen::MatrixXd& _colour);

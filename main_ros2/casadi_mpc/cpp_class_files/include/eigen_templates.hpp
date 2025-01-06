#include <Eigen/Dense>

template <typename T>
Eigen::Map<const Eigen::VectorXi> ConstIntVectorMap(T *ptr, int size)
{
    return Eigen::Map<const Eigen::VectorXi>(reinterpret_cast<int *>(const_cast<casadi_uint *>(ptr)), size);
}

template <typename T>
Eigen::Map<const Eigen::VectorXd> ConstDoubleVectorMap(T *ptr, int size)
{
    return Eigen::Map<const Eigen::VectorXd>(const_cast<double *>(ptr), size);
}
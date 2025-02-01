// CustomQuaternionPlugin.h
template <typename Derived>
EIGEN_DEVICE_FUNC inline Quaternion(const Eigen::MatrixBase<Derived>& vec)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 4);
    // Assume vec is in [w, x, y, z] order
    this->w() = vec(0);
    this->x() = vec(1);
    this->y() = vec(2);
    this->z() = vec(3);
}

EIGEN_DEVICE_FUNC inline Eigen::Vector4d vec() const {
    return Eigen::Vector4d(this->w(), this->x(), this->y(), this->z());
}
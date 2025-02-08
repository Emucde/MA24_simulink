// CustomQuaternionPlugin.h
template <typename Derived>
EIGEN_DEVICE_FUNC inline Quaternion(const Eigen::Vector4d& vec) {
    this->w() = vec(0);
    this->x() = vec(1);
    this->y() = vec(2);
    this->z() = vec(3);
}

EIGEN_DEVICE_FUNC inline Eigen::Vector4d vec4d() const {
    // Return the quaternion in [w, x, y, z] instead of [x, y, z, w]
    return Eigen::Vector4d(this->w(), this->x(), this->y(), this->z());
}
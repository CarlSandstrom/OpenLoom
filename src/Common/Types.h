#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Meshing
{

// Type aliases for clarity
using Point2D = Eigen::Vector2d;
using Point3D = Eigen::Vector3d;
using Vector2D = Eigen::Vector2d;
using Vector3D = Eigen::Vector3d;

// For parameter passing (accepts various Eigen expressions without copying)
using Point3DRef = Eigen::Ref<const Eigen::Vector3d>;
using Vector3DRef = Eigen::Ref<const Eigen::Vector3d>;

} // namespace Meshing
#include "OpenCascadeVolume.h"
#include <TopoDS_Shape.hxx>
#include <functional>
#include <sstream>

namespace Geometry3D
{

OpenCascadeVolume::OpenCascadeVolume(const TopoDS_Solid& solid) :
    solid_(solid)
{
}

std::string OpenCascadeVolume::getId() const
{
    std::ostringstream oss;
    oss << "OpenCascadeVolume_" << std::hex << std::hash<TopoDS_Shape>{}(solid_);
    return oss.str();
}

} // namespace Geometry3D

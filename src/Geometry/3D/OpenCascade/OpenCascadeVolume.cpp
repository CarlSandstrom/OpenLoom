#include "OpenCascadeVolume.h"
#include <BRepClass3d_SolidClassifier.hxx>
#include <Precision.hxx>
#include <TopAbs_State.hxx>
#include <TopoDS_Shape.hxx>
#include <functional>
#include <gp_Pnt.hxx>
#include <sstream>

namespace Geometry3D
{

OpenCascadeVolume::OpenCascadeVolume(const TopoDS_Solid& solid) :
    solid_(solid)
{
}

// Defined here so that the compiler sees the complete type of
// BRepClass3d_SolidClassifier when instantiating std::unique_ptr's destructor.
OpenCascadeVolume::~OpenCascadeVolume() = default;

std::string OpenCascadeVolume::getId() const
{
    std::ostringstream oss;
    oss << "OpenCascadeVolume_" << std::hex << std::hash<TopoDS_Shape>{}(solid_);
    return oss.str();
}

VolumeClassification OpenCascadeVolume::classifyPoint(const Meshing::Point3D& point) const
{
    if (!classifier_)
        classifier_ = std::make_unique<BRepClass3d_SolidClassifier>(solid_);

    classifier_->Perform(gp_Pnt(point.x(), point.y(), point.z()), Precision::Confusion());

    switch (classifier_->State())
    {
    case TopAbs_IN:
        return VolumeClassification::Inside;
    case TopAbs_OUT:
        return VolumeClassification::Outside;
    case TopAbs_ON:
        return VolumeClassification::OnBoundary;
    default:
        return VolumeClassification::Unknown;
    }
}

} // namespace Geometry3D

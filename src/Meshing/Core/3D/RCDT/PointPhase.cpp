#include "Meshing/Core/3D/RCDT/PointPhase.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IVolume3D.h"

#include <optional>

namespace Meshing
{

PointPhase classifyPointPhase(const Point3D& point,
                              const std::vector<std::string>& volumeIds,
                              const Geometry3D::GeometryCollection3D& geometry)
{
    std::optional<std::string> insideVolumeId;
    for (const auto& volumeId : volumeIds)
    {
        const Geometry3D::IVolume3D* volume = geometry.getVolume(volumeId);
        if (!volume)
            continue;

        switch (volume->classifyPoint(point))
        {
        case Geometry3D::VolumeClassification::Inside:
            if (insideVolumeId)
                return {PointPhaseKind::Ambiguous, {}};
            insideVolumeId = volumeId;
            break;
        case Geometry3D::VolumeClassification::OnBoundary:
        case Geometry3D::VolumeClassification::Unknown:
            return {PointPhaseKind::Ambiguous, {}};
        case Geometry3D::VolumeClassification::Outside:
            break;
        }
    }

    if (insideVolumeId)
        return {PointPhaseKind::InVolume, *insideVolumeId};
    return {PointPhaseKind::Exterior, {}};
}

} // namespace Meshing

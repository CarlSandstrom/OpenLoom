#include "Meshing/Core/3D/RCDT/RCDTQualityController.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/ElementQuality3D.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Data/2D/TriangleElement.h"

#include <cmath>
#include <numbers>

namespace Meshing
{

RCDTQualityController::RCDTQualityController(const MeshData3D& meshData,
                                             const Geometry3D::GeometryCollection3D& geometry,
                                             const SurfaceMesh3DQualitySettings& settings) :
    meshData_(&meshData),
    geometry_(&geometry),
    settings_(settings)
{
}

bool RCDTQualityController::isTriangleAcceptable(const TriangleElement& triangle,
                                                 const std::string& surfaceId) const
{
    const ElementGeometry3D elementGeometry(*meshData_);
    const ElementQuality3D elementQuality(*meshData_);

    const auto circumcircle = elementGeometry.computeCircumcircle(triangle);
    if (!circumcircle)
        return true;

    // A zero-length edge means two nodes coincide. Neither shape criterion can
    // be refined out of such a triangle, so both are skipped and it is treated
    // as acceptable rather than perpetually bad. The chord-deviation criterion
    // below still applies: it measures where the circumcenter sits relative to
    // the CAD surface, not the triangle's shape.
    const double shortestEdge = elementQuality.getShortestEdgeLength(triangle);
    if (shortestEdge > 0.0)
    {
        if (circumcircle->radius / shortestEdge > settings_.circumradiusToShortestEdgeRatio)
            return false;

        const double minAngleRadians = settings_.minAngleDegrees * (std::numbers::pi / 180.0);
        if (elementQuality.getMinAngle(triangle) < minAngleRadians)
            return false;
    }

    if (settings_.chordDeviationTolerance > 0.0)
    {
        const Geometry3D::ISurface3D* surface = geometry_->getSurface(surfaceId);
        if (surface && std::abs(SurfaceProjector::signedDistance(circumcircle->center, *surface)) >
                           settings_.chordDeviationTolerance)
            return false;
    }

    return true;
}

} // namespace Meshing

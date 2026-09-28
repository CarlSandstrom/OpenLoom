#include "Meshing/Core/3D/RCDT/SurfaceFacetCriteria.h"

#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/RCDT/ProtectionExemption.h"
#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace Meshing
{

namespace
{

constexpr int SHAPE_CRITERION = 0;
constexpr int DISTANCE_CRITERION = 1;
constexpr int SAME_PATCH_CRITERION = 2;

/// CGAL's aspect ratio: sin^2 of the smallest angle, from squared edge
/// lengths and squared area. nullopt for a triangle with a zero-length edge.
std::optional<double> aspectRatio(const FaceKey& face, const MeshData3D& meshData)
{
    const Point3D& a = meshData.getNode(face.nodeIds[0])->getCoordinates();
    const Point3D& b = meshData.getNode(face.nodeIds[1])->getCoordinates();
    const Point3D& c = meshData.getNode(face.nodeIds[2])->getCoordinates();
    const double ab = (b - a).squaredNorm();
    const double ac = (c - a).squaredNorm();
    const double bc = (c - b).squaredNorm();
    if (ab <= 0.0 || ac <= 0.0 || bc <= 0.0)
        return std::nullopt;
    const double squaredArea = 0.25 * (b - a).cross(c - a).squaredNorm();
    return 4.0 * squaredArea * std::min({ab, ac, bc}) / (ab * ac * bc);
}

} // namespace

SurfaceFacetCriteria::SurfaceFacetCriteria(const SurfaceMesh3DQualitySettings& settings,
                                           std::unordered_set<std::string> surfaceIds) :
    minimumAngleDegrees_(settings.minAngleDegrees),
    distanceBound_(settings.chordDeviationTolerance),
    surfaceIds_(std::move(surfaceIds))
{
}

std::optional<SurfaceFacetCriteria::Badness> SurfaceFacetCriteria::findBadness(const FaceKey& face,
                                                                               const RestrictedFacet& facet,
                                                                               const MeshData3D& meshData) const
{
    const TriangleElement triangle(face.nodeIds);
    const ProtectionExemption exemption = protectionExemption(triangle, meshData);

    if (!exemption.angle)
    {
        const double bound = std::sin(minimumAngleDegrees_ * std::numbers::pi / 180.0);
        if (const auto ratio = aspectRatio(face, meshData); ratio && *ratio < bound * bound)
            return Badness{SHAPE_CRITERION, *ratio};
    }

    if (!exemption.chord && distanceBound_ > 0.0)
    {
        const ElementGeometry3D elementGeometry(meshData);
        if (const auto center = elementGeometry.computeOrthocenter(triangle))
        {
            const double squaredDistance = (*center - facet.surfaceCenter).squaredNorm();
            const double squaredBound = distanceBound_ * distanceBound_;
            if (squaredDistance > squaredBound)
                return Badness{DISTANCE_CRITERION, squaredBound / squaredDistance};
        }
    }

    const std::string* patch = nullptr;
    for (const size_t nodeId : face.nodeIds)
    {
        const auto& geometryIds = meshData.getGeometryIds(nodeId);
        if (geometryIds.size() != 1 || !surfaceIds_.count(geometryIds.front()))
            continue;
        if (patch && *patch != geometryIds.front())
            return Badness{SAME_PATCH_CRITERION, 1.0};
        patch = &geometryIds.front();
    }
    return std::nullopt;
}

} // namespace Meshing

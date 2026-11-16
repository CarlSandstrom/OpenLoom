#include "Meshing/Core/3D/RCDT/SurfaceFacetCriteria.h"

#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/RCDT/ProtectionExemption.h"
#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"
#include "Meshing/Data/Base/TriangleElement.h"
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

std::optional<double> aspectRatio(const FaceKey& face, const MeshData3D& meshData);

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
    TriangleElement triangle(face.nodeIds);
    auto exemption = protectionExemption(triangle, meshData);

    if (!exemption.angle)
    {
        double bound = std::sin(minimumAngleDegrees_ * std::numbers::pi / 180.0);
        if (auto ratio = aspectRatio(face, meshData); ratio && *ratio < bound * bound)
        {
            return Badness{SHAPE_CRITERION, *ratio};
        }
    }

    if (!exemption.chord && distanceBound_ > 0.0)
    {
        ElementGeometry3D elementGeometry(meshData);
        if (auto center = elementGeometry.computeOrthocenter(triangle))
        {
            double squaredDistance = (*center - facet.surfaceCenter).squaredNorm();
            double squaredBound = distanceBound_ * distanceBound_;
            if (squaredDistance > squaredBound) return Badness{DISTANCE_CRITERION, squaredBound / squaredDistance};
        }
    }

    const std::string* patch = nullptr;
    for (auto nodeId : face.nodeIds)
    {
        auto& geometryIds = meshData.getGeometryIds(nodeId);
        if (geometryIds.size() != 1 || !surfaceIds_.count(geometryIds.front())) continue;

        if (patch && *patch != geometryIds.front()) return Badness{SAME_PATCH_CRITERION, 1.0};
        patch = &geometryIds.front();
    }
    return std::nullopt;
}

namespace
{

// CGAL's aspect ratio: sin^2 of the smallest angle, from squared edge lengths and squared area.
// nullopt for a triangle with a zero-length edge.
std::optional<double> aspectRatio(const FaceKey& face, const MeshData3D& meshData)
{
    auto& pointA = meshData.getNode(face.nodeIds[0])->getCoordinates();
    auto& pointB = meshData.getNode(face.nodeIds[1])->getCoordinates();
    auto& pointC = meshData.getNode(face.nodeIds[2])->getCoordinates();
    double squaredLengthAB = (pointB - pointA).squaredNorm();
    double squaredLengthAC = (pointC - pointA).squaredNorm();
    double squaredLengthBC = (pointC - pointB).squaredNorm();
    if (squaredLengthAB <= 0.0 || squaredLengthAC <= 0.0 || squaredLengthBC <= 0.0) return std::nullopt;

    double squaredArea = 0.25 * (pointB - pointA).cross(pointC - pointA).squaredNorm();
    return 4.0 * squaredArea * std::min({squaredLengthAB, squaredLengthAC, squaredLengthBC}) /
           (squaredLengthAB * squaredLengthAC * squaredLengthBC);
}

} // namespace

} // namespace Meshing

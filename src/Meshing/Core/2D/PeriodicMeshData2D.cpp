#include "PeriodicMeshData2D.h"

#include "Common/Exceptions/MeshException.h"
#include "Meshing/Data/2D/Node2D.h"

#include <string>

namespace Meshing
{

PeriodicMeshData2D::PeriodicMeshData2D(const MeshData2D& meshData,
                                       const PeriodicDomainConfig& config) :
    meshData_(meshData),
    config_(config),
    offsetTable_(config)
{
}

Point2D PeriodicMeshData2D::applyOffset(const Point2D& coord,
                                        const PeriodicOffset& offset) const
{
    Point2D result = coord;
    if (config_.uPeriodic)
        result.x() += offset.u * config_.uPeriod;
    if (config_.vPeriodic)
        result.y() += offset.v * config_.vPeriod;
    return result;
}

Point2D PeriodicMeshData2D::getCoordinate(size_t triangleId, int slot) const
{
    const auto* triangle = dynamic_cast<const TriangleElement*>(
        meshData_.getElement(triangleId));

    if (!triangle)
    {
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::ELEMENT_NOT_FOUND,
                            "Element " + std::to_string(triangleId) + " is not a TriangleElement");
    }

    const size_t nodeId = triangle->getNodeIdArray()[slot];
    const Point2D canonical = meshData_.getNode(nodeId)->getCoordinates();
    const auto& offsets = offsetTable_.getOffsets(triangleId);
    return applyOffset(canonical, offsets[slot]);
}

std::tuple<Point2D, Point2D, Point2D>
PeriodicMeshData2D::getTriangleCoordinates(size_t triangleId) const
{
    return {getCoordinate(triangleId, 0),
            getCoordinate(triangleId, 1),
            getCoordinate(triangleId, 2)};
}

Point2D PeriodicMeshData2D::nearestCopy(const Point2D& point,
                                        const Point2D& referencePoint) const
{
    Point2D result = point;

    if (config_.uPeriodic)
    {
        const double delta = point.x() - referencePoint.x();
        if (delta > config_.uPeriod * 0.5)
            result.x() -= config_.uPeriod;
        else if (delta < -config_.uPeriod * 0.5)
            result.x() += config_.uPeriod;
    }

    if (config_.vPeriodic)
    {
        const double delta = point.y() - referencePoint.y();
        if (delta > config_.vPeriod * 0.5)
            result.y() -= config_.vPeriod;
        else if (delta < -config_.vPeriod * 0.5)
            result.y() += config_.vPeriod;
    }

    return result;
}

} // namespace Meshing

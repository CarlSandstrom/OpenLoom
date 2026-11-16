#include "PeriodicMeshData2D.h"

#include "Common/Exceptions/MeshException.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/Base/TriangleElement.h"

#include <string>

namespace Meshing
{

PeriodicMeshData2D::PeriodicMeshData2D(const MeshData2D& meshData,
                                       const PeriodicDomainConfiguration& configuration) :
    meshData_(meshData),
    configuration_(configuration),
    offsetTable_(configuration)
{
}

Point2D PeriodicMeshData2D::applyOffset(const Point2D& coordinate,
                                        const PeriodicOffset& offset) const
{
    Point2D result = coordinate;
    if (configuration_.uPeriodic)
        result.x() += offset.u * configuration_.uPeriod;
    if (configuration_.vPeriodic)
        result.y() += offset.v * configuration_.vPeriod;
    return result;
}

Point2D PeriodicMeshData2D::getCoordinate(size_t triangleId, int slot) const
{
    auto triangle = dynamic_cast<const TriangleElement*>(
        meshData_.getElement(triangleId));

    if (!triangle)
    {
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::ELEMENT_NOT_FOUND,
                            "Element " + std::to_string(triangleId) + " is not a TriangleElement");
    }

    auto nodeId = triangle->getNodeIdArray()[slot];
    auto canonical = meshData_.getNode(nodeId)->getCoordinates();
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

namespace
{
double nearestPeriodShift(double delta, double period);
}

Point2D PeriodicMeshData2D::nearestCopy(const Point2D& point, const Point2D& referencePoint) const
{
    Point2D result = point;
    if (configuration_.uPeriodic)
        result.x() += nearestPeriodShift(point.x() - referencePoint.x(), configuration_.uPeriod);
    if (configuration_.vPeriodic)
        result.y() += nearestPeriodShift(point.y() - referencePoint.y(), configuration_.vPeriod);
    return result;
}

namespace
{

/// The shift of one period that brings a point 'delta' away from its reference
/// to within half a period; zero when it is already that close.
double nearestPeriodShift(double delta, double period)
{
    if (delta > period * 0.5)
        return -period;
    if (delta < -period * 0.5)
        return period;
    return 0.0;
}

} // namespace

} // namespace Meshing

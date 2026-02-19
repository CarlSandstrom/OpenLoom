#include "Meshing/Core/3D/General/ConstraintChecker3D.h"
#include "Meshing/Core/3D/General/GeometryUtilities3D.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Data/3D/Node3D.h"

namespace Meshing
{

ConstraintChecker3D::ConstraintChecker3D(const MeshData3D& mesh) :
    mesh_(mesh)
{
}

bool ConstraintChecker3D::isSubsegmentEncroached(const ConstrainedSubsegment3D& subsegment,
                                                 const Point3D& point) const
{
    // Get the endpoints of the subsegment
    const auto* node1 = mesh_.getNode(subsegment.nodeId1);
    const auto* node2 = mesh_.getNode(subsegment.nodeId2);

    if (!node1 || !node2)
    {
        return false;
    }

    // Create diametral sphere
    DiametralSphere sphere = GeometryUtilities3D::createDiametralSphere(
        node1->getCoordinates(),
        node2->getCoordinates());

    // Test if point is inside the diametral sphere
    return GeometryUtilities3D::isPointInDiametralSphere(sphere, point);
}

bool ConstraintChecker3D::isSubfacetEncroached(const ConstrainedSubfacet3D& subfacet,
                                               const Point3D& point) const
{
    // Get the three vertices of the subfacet
    const auto* node1 = mesh_.getNode(subfacet.nodeId1);
    const auto* node2 = mesh_.getNode(subfacet.nodeId2);
    const auto* node3 = mesh_.getNode(subfacet.nodeId3);

    if (!node1 || !node2 || !node3)
    {
        return false;
    }

    Point3D p1 = node1->getCoordinates();
    Point3D p2 = node2->getCoordinates();
    Point3D p3 = node3->getCoordinates();

    // Create equatorial sphere
    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);

    // Test if point is inside the equatorial sphere
    // This automatically checks for non-coplanarity
    return GeometryUtilities3D::isPointInEquatorialSphere(sphere, point, p1, p2, p3);
}

bool ConstraintChecker3D::isConstraintFace(size_t nodeId1,
                                           size_t nodeId2,
                                           size_t nodeId3,
                                           const std::vector<ConstrainedSubfacet3D>& constrainedSubfacets)
{
    FaceKey faceKey(nodeId1, nodeId2, nodeId3);
    for (const auto& subfacet : constrainedSubfacets)
    {
        if (faceKey == FaceKey(subfacet.nodeId1, subfacet.nodeId2, subfacet.nodeId3))
        {
            return true;
        }
    }
    return false;
}

} // namespace Meshing

#pragma once

namespace Meshing
{
class MeshData3D;
class TriangleElement;
} // namespace Meshing

namespace Meshing
{

/// Which criteria a triangle touching protecting balls is exempt from.
struct ProtectionExemption
{
    bool angle = false;
    bool chord = false;
};

/// CGAL Mesh_3's rule for triangles touching protecting balls
/// (Facet_criterion_visitor_with_features). Protection already guarantees the
/// mesh near a protected curve, so a triangle that is small relative to the
/// balls it touches is not refined for shape or chord: refining it would only
/// place points inside or against those balls. ratio compares the triangle's
/// extent beyond each weighted vertex -- the smallest sphere orthogonal to that
/// vertex and an unweighted one -- with the vertex's own ball.
ProtectionExemption protectionExemption(const TriangleElement& triangle, const MeshData3D& meshData);

} // namespace Meshing

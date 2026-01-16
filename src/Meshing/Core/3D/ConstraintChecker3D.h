#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include <array>
#include <vector>

namespace Meshing
{

/**
 * @brief Helper that provides constraint-related geometric checks for 3D meshes
 *
 * Implements Shewchuk's encroachment tests for 3D Delaunay refinement:
 * - Subsegment encroachment: Point inside diametral sphere
 * - Subfacet encroachment: Non-coplanar point inside equatorial sphere
 */
class ConstraintChecker3D
{
public:
    explicit ConstraintChecker3D(const MeshData3D& mesh);

    /**
     * @brief Tests if a subsegment is encroached by a point
     *
     * A subsegment is encroached if a point (other than its endpoints)
     * lies inside its diametral sphere.
     *
     * @param subsegment The constrained subsegment to test
     * @param point The point to test for encroachment
     * @return true if the point encroaches the subsegment
     */
    bool isSubsegmentEncroached(const ConstrainedSubsegment3D& subsegment,
                               const Point3D& point) const;

    /**
     * @brief Tests if a subfacet is encroached by a point
     *
     * A subfacet is encroached if a non-coplanar point lies inside
     * its equatorial sphere. Coplanar points do not cause encroachment.
     *
     * @param subfacet The constrained subfacet to test
     * @param point The point to test for encroachment
     * @return true if the point encroaches the subfacet
     */
    bool isSubfacetEncroached(const ConstrainedSubfacet3D& subfacet,
                             const Point3D& point) const;

    /**
     * @brief Check if a triangular face matches any constraint face in a list
     *
     * Compares a face (given by three node IDs) against a list of constrained
     * subfacets to see if it matches any of them (order-independent comparison).
     *
     * @param nodeId1 First vertex of the face
     * @param nodeId2 Second vertex of the face
     * @param nodeId3 Third vertex of the face
     * @param constrainedSubfacets List of constraint faces to check against
     * @return true if the face matches any constraint
     */
    static bool isConstraintFace(size_t nodeId1,
                                 size_t nodeId2,
                                 size_t nodeId3,
                                 const std::vector<ConstrainedSubfacet3D>& constrainedSubfacets);

private:
    const MeshData3D& mesh_;

    /**
     * @brief Create a sorted key for face comparison (order-independent)
     */
    static std::array<size_t, 3> makeFaceKey(size_t a, size_t b, size_t c);
};

} // namespace Meshing

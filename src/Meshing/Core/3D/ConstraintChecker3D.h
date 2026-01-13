#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Data/3D/MeshData3D.h"

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

private:
    const MeshData3D& mesh_;
};

} // namespace Meshing

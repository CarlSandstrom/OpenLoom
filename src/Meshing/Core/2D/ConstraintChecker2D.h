#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"

namespace Meshing
{

/// Helper that provides constraint-related geometric checks for 2D meshes.
class ConstraintChecker2D
{
public:
    explicit ConstraintChecker2D(const MeshData2D& mesh);

    /// Tests if a segment is encroached by a point.
    /// A segment is encroached if the point lies inside the diametral circle of the segment.
    bool isSegmentEncroached(const ConstrainedSegment2D& segment, const Point2D& point) const;

private:
    const MeshData2D& mesh_;
};

} // namespace Meshing

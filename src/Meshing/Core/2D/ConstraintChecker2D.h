#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"

namespace Meshing
{

class PeriodicMeshData2D;

/// Helper that provides constraint-related geometric checks for 2D meshes.
///
/// When a PeriodicMeshData2D pointer is provided, isSegmentEncroached shifts
/// the candidate point into the same periodic frame as the segment before
/// testing the diametral circle, so that points near the opposite period
/// boundary are correctly detected as encroaching.
class ConstraintChecker2D
{
public:
    explicit ConstraintChecker2D(const MeshData2D& mesh);
    ConstraintChecker2D(const MeshData2D& mesh, PeriodicMeshData2D* periodicData);

    /// Tests if a segment is encroached by a point.
    /// A segment is encroached if the point lies inside the diametral circle of the segment.
    bool isSegmentEncroached(const ConstrainedSegment2D& segment, const Point2D& point) const;

private:
    const MeshData2D& mesh_;
    PeriodicMeshData2D* periodicData_ = nullptr;
};

} // namespace Meshing

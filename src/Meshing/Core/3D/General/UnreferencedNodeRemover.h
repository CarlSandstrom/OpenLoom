#pragma once

#include "Common/Types.h"

#include <vector>

namespace Meshing
{

struct SurfaceMesh3D;
struct VolumeMesh3D;

/**
 * @brief Drops the nodes of a finished mesh that nothing references, and
 * renumbers the rest densely.
 *
 * A mesher that builds its output from a working triangulation keeps that
 * triangulation's node IDs, so the output carries every node the working mesh
 * ever had: slots left by removed bounding vertices, and vertices no output
 * cell uses. A node counts as referenced if a cell or an edge node chain names
 * it. Surviving nodes keep their relative order, and every node ID in the mesh
 * is rewritten to the new numbering.
 *
 * Both overloads return that numbering, indexed by old node ID, so a caller
 * holding IDs of the working mesh can translate them. A dropped node maps to
 * NodeId::INVALID.
 */
class UnreferencedNodeRemover
{
public:
    static std::vector<NodeId> remove(SurfaceMesh3D& surfaceMesh);
    static std::vector<NodeId> remove(VolumeMesh3D& volumeMesh);
};

} // namespace Meshing

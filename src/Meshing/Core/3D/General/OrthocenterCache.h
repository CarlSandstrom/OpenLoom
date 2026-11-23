#pragma once

#include "Common/Types.h"

#include <array>
#include <optional>
#include <unordered_map>

namespace Meshing
{

class MeshData3D;
class TetrahedronElement;

/**
 * @brief Tetrahedron orthocenters, computed once and reused while unchanged.
 *
 * ElementGeometry3D::computeOrthocenter falls back to exact arithmetic for a
 * near-flat tetrahedron, on the order of a thousand times the cost of the
 * certified double solve, and restriction asks for the same tetrahedra's
 * orthocenters every time one of their faces is reclassified (OPE-248).
 *
 * An entry remembers the four vertices' coordinates and weights it was computed
 * from and is returned only while they are bitwise the same, so the cache needs
 * no notice of mutations: a vertex moved by smoothing or given a new weight by
 * sliver exudation simply misses. Element ids are never reused, so an id names
 * one tetrahedron for good; entries of removed tetrahedra are swept out once the
 * cache holds twice as many entries as the mesh has elements.
 */
class OrthocenterCache
{
public:
    std::optional<Point3D> orthocenterOf(ElementId elementId,
                                         const TetrahedronElement& tetrahedron,
                                         const MeshData3D& meshData);

private:
    struct VertexState
    {
        Point3D coordinates;
        double weight;

        bool operator==(const VertexState& other) const = default;
    };

    struct Entry
    {
        std::array<VertexState, 4> vertices;
        std::optional<Point3D> orthocenter;
    };

    void sweepRemovedElements(const MeshData3D& meshData);

    std::unordered_map<ElementId, Entry> entries_;
};

} // namespace Meshing

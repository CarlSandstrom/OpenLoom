#include "Meshing/Core/3D/General/OrthocenterCache.h"

#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"

namespace Meshing
{

std::optional<Point3D> OrthocenterCache::orthocenterOf(ElementId elementId,
                                                       const TetrahedronElement& tetrahedron,
                                                       const MeshData3D& meshData)
{
    std::array<VertexState, 4> vertices;
    auto& nodeIds = tetrahedron.getNodeIds();
    for (size_t i = 0; i < 4; ++i)
    {
        auto node = meshData.getNode(nodeIds[i]);
        vertices[i] = {node->getCoordinates(), node->getWeight()};
    }

    auto found = entries_.find(elementId);
    if (found != entries_.end() && found->second.vertices == vertices) return found->second.orthocenter;

    auto orthocenter = ElementGeometry3D(meshData).computeOrthocenter(tetrahedron);
    entries_[elementId] = {vertices, orthocenter};
    if (entries_.size() > 2 * meshData.getElementCount()) sweepRemovedElements(meshData);
    return orthocenter;
}

void OrthocenterCache::sweepRemovedElements(const MeshData3D& meshData)
{
    std::erase_if(entries_, [&meshData](const auto& entry)
                  { return meshData.getElement(entry.first) == nullptr; });
}

} // namespace Meshing

#pragma once

#include "../Base/IElement.h"
#include "MeshData2D.h"
#include <memory>
#include <string>

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief Operations for modifying 2D mesh data
 */
class MeshMutator2D
{
public:
    explicit MeshMutator2D(MeshData2D& meshData);

    // Node operations
    size_t addNode(const Point2D& coordinates);
    size_t addBoundaryNode(const Point2D& coordinates, double edgeParameter, const std::string& geometryId);
    void moveNode(size_t id, const Point2D& newCoords);
    void removeNode(size_t id);

    // Element operations
    size_t addElement(std::unique_ptr<IElement> element);
    void removeElement(size_t id);

private:
    MeshData2D& meshData_;
};

} // namespace Meshing

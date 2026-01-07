#include "HoleTriangleRemover.h"

#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/Base/IElement.h"
#include "spdlog/spdlog.h"
#include <vector>

namespace Meshing
{

HoleTriangleRemover::HoleTriangleRemover(MeshData2D& meshData,
                                         MeshMutator2D& mutator,
                                         const Geometry2D::IFace2D& face) :
    meshData_(meshData),
    mutator_(mutator),
    face_(face),
    removedCount_(0)
{
}

void HoleTriangleRemover::removeInvalidTriangles()
{
    // Collect element IDs to remove (can't modify while iterating)
    std::vector<size_t> elementsToRemove;

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        Point2D centroid = computeCentroid(elemId);
        Geometry2D::PointLocation location = face_.classify(centroid);

        switch (location)
        {
        case Geometry2D::PointLocation::Inside:
            break;
        case Geometry2D::PointLocation::Outside:
            elementsToRemove.push_back(elemId);
            break;
        case Geometry2D::PointLocation::OnBoundary:
            break;
        }
    }

    // Remove the collected elements
    for (size_t elemId : elementsToRemove)
    {
        mutator_.removeElement(elemId);
    }

    removedCount_ = elementsToRemove.size();

    spdlog::debug("HoleTriangleRemover: removed {} triangles", removedCount_);
}

Point2D HoleTriangleRemover::computeCentroid(size_t elemId) const
{
    const IElement* element = meshData_.getElement(elemId);
    if (!element)
    {
        return Point2D(0.0, 0.0);
    }

    const std::vector<size_t>& nodeIds = element->getNodeIds();

    double sumX = 0.0;
    double sumY = 0.0;

    for (size_t nodeId : nodeIds)
    {
        const Node2D* node = meshData_.getNode(nodeId);
        if (node)
        {
            Point2D coords = node->getCoordinates();
            sumX += coords.x();
            sumY += coords.y();
        }
    }

    size_t count = nodeIds.size();
    return Point2D(sumX / count, sumY / count);
}

} // namespace Meshing

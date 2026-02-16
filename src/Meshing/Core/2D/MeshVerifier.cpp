#include "MeshVerifier.h"
#include "GeometryUtilities2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>

namespace Meshing
{

MeshVerifier::MeshVerifier(const MeshData2D& meshData) :
    meshData_(meshData)
{
#ifndef CMESH_HAS_OPENMP
    static bool warned = false;
    if (!warned)
    {
        spdlog::warn("MeshVerifier: OpenMP is disabled. Overlap checks will run sequentially. "
                     "Enable with: cmake -DCMESH_USE_OPENMP=ON");
        warned = true;
    }
#endif
}

MeshVerifier::VerificationResult MeshVerifier::verify() const
{
    VerificationResult result;
    result.isValid = true;

    // Check orientation
    for (const auto& [id, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
        {
            result.warnings.push_back("Element " + std::to_string(id) + " is not a triangle, skipping");
            continue;
        }

        const auto& nodeIds = triangle->getNodeIdArray();
        const Point2D& p1 = meshData_.getNode(nodeIds[0])->getCoordinates();
        const Point2D& p2 = meshData_.getNode(nodeIds[1])->getCoordinates();
        const Point2D& p3 = meshData_.getNode(nodeIds[2])->getCoordinates();

        double area = GeometryUtilities2D::computeSignedArea(p1, p2, p3);
        if (area < -1e-10)
        {
            result.isValid = false;
            result.errors.push_back("Element " + std::to_string(id) +
                                    " has clockwise orientation (signed area: " +
                                    std::to_string(area) + ")");
        }
        else if (std::abs(area) < 1e-10)
        {
            result.isValid = false;
            result.errors.push_back("Element " + std::to_string(id) +
                                    " is degenerate (area near zero)");
        }
    }

    // Check for overlaps
    std::vector<size_t> elementIds;
    std::vector<std::array<Point2D, 3>> triangleCoords;

    for (const auto& [id, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
        {
            continue;
        }

        const auto& nodeIds = triangle->getNodeIdArray();
        std::array<Point2D, 3> coords = {
            meshData_.getNode(nodeIds[0])->getCoordinates(),
            meshData_.getNode(nodeIds[1])->getCoordinates(),
            meshData_.getNode(nodeIds[2])->getCoordinates()};

        elementIds.push_back(id);
        triangleCoords.push_back(coords);
    }

    // Check all pairs of triangles for overlap
#ifdef CMESH_HAS_OPENMP
    #pragma omp parallel
    {
        std::vector<std::string> localErrors;

        #pragma omp for schedule(dynamic)
        for (size_t i = 0; i < triangleCoords.size(); ++i)
        {
            for (size_t j = i + 1; j < triangleCoords.size(); ++j)
            {
                if (trianglesOverlap(triangleCoords[i], triangleCoords[j]))
                {
                    localErrors.push_back("Elements " + std::to_string(elementIds[i]) +
                                          " and " + std::to_string(elementIds[j]) +
                                          " overlap");
                }
            }
        }

        if (!localErrors.empty())
        {
            #pragma omp critical
            {
                result.isValid = false;
                result.errors.insert(result.errors.end(), localErrors.begin(), localErrors.end());
            }
        }
    }
#else
    for (size_t i = 0; i < triangleCoords.size(); ++i)
    {
        for (size_t j = i + 1; j < triangleCoords.size(); ++j)
        {
            if (trianglesOverlap(triangleCoords[i], triangleCoords[j]))
            {
                result.isValid = false;
                result.errors.push_back("Elements " + std::to_string(elementIds[i]) +
                                        " and " + std::to_string(elementIds[j]) +
                                        " overlap");
            }
        }
    }
#endif

    if (result.isValid)
    {
        SPDLOG_INFO("Mesh verification passed: {} elements verified", meshData_.getElementCount());
    }
    else
    {
        SPDLOG_ERROR("Mesh verification failed with {} errors", result.errors.size());
    }

    return result;
}

bool MeshVerifier::trianglesOverlap(const std::array<Point2D, 3>& tri1Nodes,
                                    const std::array<Point2D, 3>& tri2Nodes)
{
    // Two triangles overlap if:
    // 1. Any vertex of one triangle is inside the other triangle
    // 2. Any edges of the triangles intersect (excluding shared edges/vertices)

    // Check if any vertex of tri1 is strictly inside tri2
    for (size_t i = 0; i < 3; ++i)
    {
        if (GeometryUtilities2D::isPointStrictlyInsideTriangle(
                tri1Nodes[i], tri2Nodes[0], tri2Nodes[1], tri2Nodes[2]))
        {
            return true;
        }
    }

    // Check if any vertex of tri2 is strictly inside tri1
    for (size_t i = 0; i < 3; ++i)
    {
        if (GeometryUtilities2D::isPointStrictlyInsideTriangle(
                tri2Nodes[i], tri1Nodes[0], tri1Nodes[1], tri1Nodes[2]))
        {
            return true;
        }
    }

    // Check if any edges intersect (excluding shared endpoints)
    for (size_t i = 0; i < 3; ++i)
    {
        const Point2D& a1 = tri1Nodes[i];
        const Point2D& a2 = tri1Nodes[(i + 1) % 3];

        for (size_t j = 0; j < 3; ++j)
        {
            const Point2D& b1 = tri2Nodes[j];
            const Point2D& b2 = tri2Nodes[(j + 1) % 3];

            if (GeometryUtilities2D::segmentsIntersectExcludingSharedEndpoints(a1, a2, b1, b2))
            {
                return true;
            }
        }
    }

    return false;
}

} // namespace Meshing

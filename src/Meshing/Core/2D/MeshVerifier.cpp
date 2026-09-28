#include "MeshVerifier.h"
#include "GeometryUtilities2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>

namespace Meshing
{

MeshVerifier::VerificationResult MeshVerifier::verify(const MeshData2D& meshData)
{
#ifndef OPENLOOM_HAS_OPENMP
    static bool warned = false;
    if (!warned)
    {
        spdlog::warn("MeshVerifier: OpenMP is disabled. Overlap checks will run sequentially. "
                     "Enable with: cmake -DOPENLOOM_USE_OPENMP=ON");
        warned = true;
    }
#endif

    VerificationResult result;
    result.isValid = true;

    // Check orientation
    for (const auto& [id, element] : meshData.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
        {
            result.warnings.push_back("Element " + std::to_string(id) + " is not a triangle, skipping");
            continue;
        }

        const auto& nodeIds = triangle->getNodeIdArray();
        const Point2D& p1 = meshData.getNode(nodeIds[0])->getCoordinates();
        const Point2D& p2 = meshData.getNode(nodeIds[1])->getCoordinates();
        const Point2D& p3 = meshData.getNode(nodeIds[2])->getCoordinates();

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
    std::vector<std::array<Point2D, 3>> triangleCoordinates;

    for (const auto& [id, element] : meshData.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
        {
            continue;
        }

        const auto& nodeIds = triangle->getNodeIdArray();
        std::array<Point2D, 3> coordinates = {
            meshData.getNode(nodeIds[0])->getCoordinates(),
            meshData.getNode(nodeIds[1])->getCoordinates(),
            meshData.getNode(nodeIds[2])->getCoordinates()};

        elementIds.push_back(id);
        triangleCoordinates.push_back(coordinates);
    }

    // Check all pairs of triangles for overlap
#ifdef OPENLOOM_HAS_OPENMP
    #pragma omp parallel
    {
        std::vector<std::string> localErrors;

        #pragma omp for schedule(dynamic)
        for (size_t i = 0; i < triangleCoordinates.size(); ++i)
        {
            for (size_t j = i + 1; j < triangleCoordinates.size(); ++j)
            {
                if (trianglesOverlap(triangleCoordinates[i], triangleCoordinates[j]))
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
    for (size_t i = 0; i < triangleCoordinates.size(); ++i)
    {
        for (size_t j = i + 1; j < triangleCoordinates.size(); ++j)
        {
            if (trianglesOverlap(triangleCoordinates[i], triangleCoordinates[j]))
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
        SPDLOG_INFO("Mesh verification passed: {} elements verified", meshData.getElementCount());
    }
    else
    {
        SPDLOG_ERROR("Mesh verification failed with {} errors", result.errors.size());
    }

    return result;
}

bool MeshVerifier::trianglesOverlap(const std::array<Point2D, 3>& triangle1Nodes,
                                    const std::array<Point2D, 3>& triangle2Nodes)
{
    // Two triangles overlap if:
    // 1. Any vertex of one triangle is inside the other triangle
    // 2. Any edges of the triangles intersect (excluding shared edges/vertices)

    // Check if any vertex of triangle1 is strictly inside triangle2
    for (size_t i = 0; i < 3; ++i)
    {
        if (GeometryUtilities2D::isPointStrictlyInsideTriangle(
                triangle1Nodes[i], triangle2Nodes[0], triangle2Nodes[1], triangle2Nodes[2]))
        {
            return true;
        }
    }

    // Check if any vertex of triangle2 is strictly inside triangle1
    for (size_t i = 0; i < 3; ++i)
    {
        if (GeometryUtilities2D::isPointStrictlyInsideTriangle(
                triangle2Nodes[i], triangle1Nodes[0], triangle1Nodes[1], triangle1Nodes[2]))
        {
            return true;
        }
    }

    // Check if any edges intersect (excluding shared endpoints)
    for (size_t i = 0; i < 3; ++i)
    {
        const Point2D& a1 = triangle1Nodes[i];
        const Point2D& a2 = triangle1Nodes[(i + 1) % 3];

        for (size_t j = 0; j < 3; ++j)
        {
            const Point2D& b1 = triangle2Nodes[j];
            const Point2D& b2 = triangle2Nodes[(j + 1) % 3];

            if (GeometryUtilities2D::segmentsIntersectExcludingSharedEndpoints(a1, a2, b1, b2))
            {
                return true;
            }
        }
    }

    return false;
}

} // namespace Meshing

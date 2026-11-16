#include "Meshing/Core/2D/General/MeshVerifier2D.h"
#include "Common/Types.h"
#include "Meshing/Core/2D/General/GeometryUtilities2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/Base/TriangleElement.h"
#include "spdlog/spdlog.h"
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace Meshing
{

namespace
{

// A signed area below minus this is a clockwise triangle; a magnitude below it is a degenerate one.
constexpr double DEGENERATE_AREA_TOLERANCE = 1e-10;

// The triangular elements of a mesh, as parallel arrays of element id and vertex coordinates.
struct TriangleCoordinates
{
    std::vector<ElementId> elementIds;
    std::vector<std::array<Point2D, 3>> vertices;
};

TriangleCoordinates collectTriangleCoordinates(const MeshData2D& meshData, MeshVerifier2D::VerificationResult& result);
void checkOrientation(const TriangleCoordinates& triangles, MeshVerifier2D::VerificationResult& result);
void checkOverlaps(const TriangleCoordinates& triangles, MeshVerifier2D::VerificationResult& result);
std::string describeOverlap(ElementId elementId1, ElementId elementId2);
bool trianglesOverlap(const std::array<Point2D, 3>& triangle1Nodes, const std::array<Point2D, 3>& triangle2Nodes);

} // namespace

MeshVerifier2D::VerificationResult MeshVerifier2D::verify(const MeshData2D& meshData)
{
#ifndef OPENLOOM_HAS_OPENMP
    static bool warned = false;
    if (!warned)
    {
        spdlog::warn("MeshVerifier2D: OpenMP is disabled. Overlap checks will run sequentially. "
                     "Enable with: cmake -DOPENLOOM_USE_OPENMP=ON");
        warned = true;
    }
#endif

    VerificationResult result;
    result.isValid = true;

    auto triangles = collectTriangleCoordinates(meshData, result);
    checkOrientation(triangles, result);
    checkOverlaps(triangles, result);

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

namespace
{

TriangleCoordinates collectTriangleCoordinates(const MeshData2D& meshData, MeshVerifier2D::VerificationResult& result)
{
    TriangleCoordinates triangles;
    for (const auto& [id, element] : meshData.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
        {
            result.warnings.push_back("Element " + std::to_string(id) + " is not a triangle, skipping");
            continue;
        }

        auto& nodeIds = triangle->getNodeIdArray();
        triangles.elementIds.push_back(id);
        triangles.vertices.push_back({meshData.getNode(nodeIds[0])->getCoordinates(),
                                      meshData.getNode(nodeIds[1])->getCoordinates(),
                                      meshData.getNode(nodeIds[2])->getCoordinates()});
    }
    return triangles;
}

void checkOrientation(const TriangleCoordinates& triangles, MeshVerifier2D::VerificationResult& result)
{
    for (size_t i = 0; i < triangles.vertices.size(); ++i)
    {
        auto id = triangles.elementIds[i];
        auto& vertices = triangles.vertices[i];
        auto area = GeometryUtilities2D::computeSignedArea(vertices[0], vertices[1], vertices[2]);
        if (area < -DEGENERATE_AREA_TOLERANCE)
        {
            result.isValid = false;
            result.errors.push_back("Element " + std::to_string(id) + " has clockwise orientation (signed area: " +
                                    std::to_string(area) + ")");
        }
        else if (std::abs(area) < DEGENERATE_AREA_TOLERANCE)
        {
            result.isValid = false;
            result.errors.push_back("Element " + std::to_string(id) + " is degenerate (area near zero)");
        }
    }
}

void checkOverlaps(const TriangleCoordinates& triangles, MeshVerifier2D::VerificationResult& result)
{
    auto& vertices = triangles.vertices;
    auto& elementIds = triangles.elementIds;

#ifdef OPENLOOM_HAS_OPENMP
#pragma omp parallel
    {
        std::vector<std::string> localErrors;

#pragma omp for schedule(dynamic)
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            for (size_t j = i + 1; j < vertices.size(); ++j)
            {
                if (trianglesOverlap(vertices[i], vertices[j]))
                {
                    localErrors.push_back(describeOverlap(elementIds[i], elementIds[j]));
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
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        for (size_t j = i + 1; j < vertices.size(); ++j)
        {
            if (!trianglesOverlap(vertices[i], vertices[j])) continue;

            result.isValid = false;
            result.errors.push_back(describeOverlap(elementIds[i], elementIds[j]));
        }
    }
#endif
}

std::string describeOverlap(ElementId elementId1, ElementId elementId2)
{
    return "Elements " + std::to_string(elementId1) + " and " + std::to_string(elementId2) + " overlap";
}

// Two triangles overlap if a vertex of one lies strictly inside the other, or if their edges cross
// anywhere other than at a shared endpoint.
bool trianglesOverlap(const std::array<Point2D, 3>& triangle1Nodes, const std::array<Point2D, 3>& triangle2Nodes)
{
    for (size_t i = 0; i < 3; ++i)
    {
        if (GeometryUtilities2D::isPointStrictlyInsideTriangle(
                triangle1Nodes[i], triangle2Nodes[0], triangle2Nodes[1], triangle2Nodes[2]))
        {
            return true;
        }
    }

    for (size_t i = 0; i < 3; ++i)
    {
        if (GeometryUtilities2D::isPointStrictlyInsideTriangle(
                triangle2Nodes[i], triangle1Nodes[0], triangle1Nodes[1], triangle1Nodes[2]))
        {
            return true;
        }
    }

    for (size_t i = 0; i < 3; ++i)
    {
        auto& edge1Start = triangle1Nodes[i];
        auto& edge1End = triangle1Nodes[(i + 1) % 3];

        for (size_t j = 0; j < 3; ++j)
        {
            auto& edge2Start = triangle2Nodes[j];
            auto& edge2End = triangle2Nodes[(j + 1) % 3];

            if (GeometryUtilities2D::segmentsIntersectExcludingSharedEndpoints(edge1Start, edge1End, edge2Start,
                                                                               edge2End))
            {
                return true;
            }
        }
    }

    return false;
}

} // namespace

} // namespace Meshing

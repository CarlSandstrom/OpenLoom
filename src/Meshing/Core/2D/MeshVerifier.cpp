#include "MeshVerifier.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>

namespace Meshing
{

MeshVerifier::MeshVerifier(const MeshData2D& meshData) :
    meshData_(meshData)
{
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

        double area = computeSignedArea(p1, p2, p3);
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

    // Check all pairs of triangles
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

bool MeshVerifier::verifyOrientation() const
{
    for (const auto& [id, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
        {
            continue;
        }

        const auto& nodeIds = triangle->getNodeIdArray();
        const Point2D& p1 = meshData_.getNode(nodeIds[0])->getCoordinates();
        const Point2D& p2 = meshData_.getNode(nodeIds[1])->getCoordinates();
        const Point2D& p3 = meshData_.getNode(nodeIds[2])->getCoordinates();

        double area = computeSignedArea(p1, p2, p3);
        if (area <= 1e-10)
        {
            return false;
        }
    }

    return true;
}

bool MeshVerifier::verifyNoOverlaps() const
{
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

        triangleCoords.push_back(coords);
    }

    for (size_t i = 0; i < triangleCoords.size(); ++i)
    {
        for (size_t j = i + 1; j < triangleCoords.size(); ++j)
        {
            if (trianglesOverlap(triangleCoords[i], triangleCoords[j]))
            {
                return false;
            }
        }
    }

    return true;
}

double MeshVerifier::computeSignedArea(const Point2D& p1, const Point2D& p2, const Point2D& p3)
{
    // Using the cross product formula: 0.5 * ((p2-p1) x (p3-p1))
    // Positive area means counter-clockwise orientation
    return 0.5 * ((p2.x() - p1.x()) * (p3.y() - p1.y()) -
                  (p3.x() - p1.x()) * (p2.y() - p1.y()));
}

bool MeshVerifier::trianglesOverlap(const std::array<Point2D, 3>& tri1Nodes,
                                    const std::array<Point2D, 3>& tri2Nodes)
{
    // Two triangles overlap if:
    // 1. Any vertex of one triangle is inside the other triangle
    // 2. Any edges of the triangles intersect (excluding shared edges/vertices)

    // Check if any vertex of tri1 is inside tri2
    for (size_t i = 0; i < 3; ++i)
    {
        if (isPointInsideTriangle(tri1Nodes[i], tri2Nodes))
        {
            return true;
        }
    }

    // Check if any vertex of tri2 is inside tri1
    for (size_t i = 0; i < 3; ++i)
    {
        if (isPointInsideTriangle(tri2Nodes[i], tri1Nodes))
        {
            return true;
        }
    }

    // Check if any edges intersect
    for (size_t i = 0; i < 3; ++i)
    {
        const Point2D& a1 = tri1Nodes[i];
        const Point2D& a2 = tri1Nodes[(i + 1) % 3];

        for (size_t j = 0; j < 3; ++j)
        {
            const Point2D& b1 = tri2Nodes[j];
            const Point2D& b2 = tri2Nodes[(j + 1) % 3];

            if (segmentsIntersect(a1, a2, b1, b2))
            {
                return true;
            }
        }
    }

    return false;
}

bool MeshVerifier::isPointInsideTriangle(const Point2D& point, const std::array<Point2D, 3>& tri)
{
    // Use barycentric coordinates method
    // A point is inside a triangle if all barycentric coordinates are non-negative
    // and their sum is <= 1

    const Point2D& v0 = tri[0];
    const Point2D& v1 = tri[1];
    const Point2D& v2 = tri[2];

    // Compute vectors
    double v0x = v1.x() - v0.x();
    double v0y = v1.y() - v0.y();
    double v1x = v2.x() - v0.x();
    double v1y = v2.y() - v0.y();
    double v2x = point.x() - v0.x();
    double v2y = point.y() - v0.y();

    // Compute dot products
    double dot00 = v0x * v0x + v0y * v0y;
    double dot01 = v0x * v1x + v0y * v1y;
    double dot02 = v0x * v2x + v0y * v2y;
    double dot11 = v1x * v1x + v1y * v1y;
    double dot12 = v1x * v2x + v1y * v2y;

    // Compute barycentric coordinates
    double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check if point is strictly inside triangle (excluding boundary)
    // This is important to avoid flagging adjacent triangles as overlapping
    const double tolerance = 1e-10;
    return (u > tolerance) && (v > tolerance) && (u + v < 1.0 - tolerance);
}

bool MeshVerifier::segmentsIntersect(const Point2D& a1, const Point2D& a2,
                                     const Point2D& b1, const Point2D& b2)
{
    // Compute orientation of ordered triplet (p, q, r)
    auto orientation = [](const Point2D& p, const Point2D& q, const Point2D& r) -> int
    {
        double val = (q.y() - p.y()) * (r.x() - q.x()) -
                     (q.x() - p.x()) * (r.y() - q.y());

        if (std::abs(val) < 1e-10) return 0; // Collinear
        return (val > 0) ? 1 : 2;            // Clockwise or counterclockwise
    };

    // Check if point q lies on segment pr
    auto onSegment = [](const Point2D& p, const Point2D& q, const Point2D& r) -> bool
    {
        return q.x() <= std::max(p.x(), r.x()) + 1e-10 &&
               q.x() >= std::min(p.x(), r.x()) - 1e-10 &&
               q.y() <= std::max(p.y(), r.y()) + 1e-10 &&
               q.y() >= std::min(p.y(), r.y()) - 1e-10;
    };

    int o1 = orientation(a1, a2, b1);
    int o2 = orientation(a1, a2, b2);
    int o3 = orientation(b1, b2, a1);
    int o4 = orientation(b1, b2, a2);

    // General case - segments intersect if they straddle each other
    if (o1 != o2 && o3 != o4)
    {
        // Exclude the case where segments only touch at endpoints
        const double tolerance = 1e-10;
        bool shareEndpoint =
            (a1 - b1).norm() < tolerance || (a1 - b2).norm() < tolerance ||
            (a2 - b1).norm() < tolerance || (a2 - b2).norm() < tolerance;

        if (!shareEndpoint)
        {
            return true;
        }
    }

    // Special cases for collinear points
    // We only consider it an intersection if segments overlap in their interior
    if (o1 == 0 && onSegment(a1, b1, a2))
    {
        // b1 is on segment a1-a2
        const double tolerance = 1e-10;
        if ((b1 - a1).norm() > tolerance && (b1 - a2).norm() > tolerance)
        {
            return true;
        }
    }

    if (o2 == 0 && onSegment(a1, b2, a2))
    {
        // b2 is on segment a1-a2
        const double tolerance = 1e-10;
        if ((b2 - a1).norm() > tolerance && (b2 - a2).norm() > tolerance)
        {
            return true;
        }
    }

    if (o3 == 0 && onSegment(b1, a1, b2))
    {
        // a1 is on segment b1-b2
        const double tolerance = 1e-10;
        if ((a1 - b1).norm() > tolerance && (a1 - b2).norm() > tolerance)
        {
            return true;
        }
    }

    if (o4 == 0 && onSegment(b1, a2, b2))
    {
        // a2 is on segment b1-b2
        const double tolerance = 1e-10;
        if ((a2 - b1).norm() > tolerance && (a2 - b2).norm() > tolerance)
        {
            return true;
        }
    }

    return false;
}

int MeshVerifier::sign(double val, double tolerance)
{
    if (val > tolerance) return 1;
    if (val < -tolerance) return -1;
    return 0;
}

} // namespace Meshing

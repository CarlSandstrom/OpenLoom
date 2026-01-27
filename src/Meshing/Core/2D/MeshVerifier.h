#pragma once

#include "Common/Types.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include <string>
#include <vector>

namespace Meshing
{

/**
 * @brief Verifies the validity of a 2D mesh
 *
 * Checks that:
 * - All triangular elements have counter-clockwise orientation
 * - No elements overlap
 */
class MeshVerifier
{
public:
    struct VerificationResult
    {
        bool isValid;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
    };

    /**
     * @brief Construct mesh verifier with mesh data
     * @param meshData Reference to the 2D mesh data to verify
     */
    explicit MeshVerifier(const MeshData2D& meshData);

    /**
     * @brief Verify the complete mesh
     * @return Verification result with validity status and error/warning messages
     */
    VerificationResult verify() const;

private:
    const MeshData2D& meshData_;

    /**
     * @brief Check if two triangles overlap
     * @param tri1Nodes Coordinates of first triangle's vertices
     * @param tri2Nodes Coordinates of second triangle's vertices
     * @return True if triangles overlap
     */
    static bool trianglesOverlap(const std::array<Point2D, 3>& tri1Nodes,
                                 const std::array<Point2D, 3>& tri2Nodes);

    /**
     * @brief Check if a point is inside or on a triangle
     * @param point Point to test
     * @param tri Triangle vertices
     * @return True if point is inside or on the triangle
     */
    static bool isPointInsideTriangle(const Point2D& point, const std::array<Point2D, 3>& tri);

    /**
     * @brief Check if two line segments intersect (excluding shared endpoints)
     * @param a1 First point of first segment
     * @param a2 Second point of first segment
     * @param b1 First point of second segment
     * @param b2 Second point of second segment
     * @return True if segments intersect in their interior
     */
    static bool segmentsIntersect(const Point2D& a1, const Point2D& a2,
                                  const Point2D& b1, const Point2D& b2);
};

} // namespace Meshing

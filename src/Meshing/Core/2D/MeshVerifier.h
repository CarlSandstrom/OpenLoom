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

    /**
     * @brief Check if all elements have counter-clockwise orientation
     * @return True if all elements are correctly oriented
     */
    bool verifyOrientation() const;

    /**
     * @brief Check if any elements overlap
     * @return True if no elements overlap
     */
    bool verifyNoOverlaps() const;

private:
    const MeshData2D& meshData_;

    /**
     * @brief Compute the signed area of a triangle
     * @param p1 First vertex
     * @param p2 Second vertex
     * @param p3 Third vertex
     * @return Signed area (positive for CCW, negative for CW)
     */
    static double computeSignedArea(const Point2D& p1, const Point2D& p2, const Point2D& p3);

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
     * @brief Check if two line segments intersect
     * @param a1 First point of first segment
     * @param a2 Second point of first segment
     * @param b1 First point of second segment
     * @param b2 Second point of second segment
     * @return True if segments intersect (excluding endpoints)
     */
    static bool segmentsIntersect(const Point2D& a1, const Point2D& a2,
                                  const Point2D& b1, const Point2D& b2);

    /**
     * @brief Compute the sign of a value with tolerance
     * @param val Value to check
     * @param tolerance Numerical tolerance
     * @return 1 for positive, -1 for negative, 0 for near-zero
     */
    static int sign(double val, double tolerance = 1e-10);
};

} // namespace Meshing

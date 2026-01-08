#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <optional>
#include <unordered_map>

namespace Meshing
{

/// Helper that owns 2D mesh data and exposes computations that require node coordinates.
class Computer2D
{
public:
    explicit Computer2D(const MeshData2D& mesh);

    // Methods that use mesh data
    std::optional<CircumCircle2D> computeCircumcircle(const TriangleElement& tri) const;
    double computeArea(const TriangleElement& element) const;
    std::array<Point2D, 3> createSuperTriangle(const std::vector<Point2D>& points);
    double computeShortestEdgeLength(const TriangleElement& element) const;
    double computeLongestEdgeLength(const TriangleElement& element) const;
    std::optional<double> computeCircumradiusToShortestEdgeRatio(const TriangleElement& element) const;
    std::array<double, 3> computeTriangleAngles(const TriangleElement& element) const;
    double computeMinAngle(const TriangleElement& element) const;
    bool isSegmentEncroached(const ConstrainedSegment2D& segment, const Point2D& point) const;
    std::optional<Point2D> computeCircumcenter(const TriangleElement& element) const;
    std::vector<size_t> getTrianglesSortedByQuality() const;

    // Static geometric utilities
    static bool isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point);
    static double computeEdgeLength(const Point2D& p1, const Point2D& p2);
    static DiametralCircle2D createDiametralCircle(const Point2D& p1, const Point2D& p2);
    static bool isPointInDiametralCircle(const DiametralCircle2D& circle, const Point2D& point);

private:
    std::tuple<Point2D, Point2D, Point2D> getElementNodeCoordinates(const TriangleElement& element) const;

    const MeshData2D& mesh_;
};

} // namespace Meshing

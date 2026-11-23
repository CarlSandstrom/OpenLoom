#pragma once

#include "Meshing/Core/2D/General/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshQualitySettings2D.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Meshing/Interfaces/2D/IQualityController2D.h"
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>

namespace Meshing
{

class TriangleElement;

/**
 * @brief Implements Shewchuk's Delaunay refinement algorithm for 2D meshes
 *
 * This class refines a 2D Delaunay mesh by:
 * 1. Splitting encroached constrained segments
 * 2. Inserting circumcenters of poor-quality triangles
 *
 * The algorithm ensures that inserting a circumcenter doesn't encroach
 * any segments by recursively splitting encroached segments first.
 *
 * Reference: "Delaunay Refinement Algorithms for Triangular Mesh Generation"
 *            by Jonathan Richard Shewchuk (2002)
 */
class ShewchukRefiner2D
{
public:
    /**
     * @brief Construct a Shewchuk refiner with quality settings.
     *
     * Creates a ShewchukQualityController2D internally from the given settings.
     * This is the primary constructor for application code.
     *
     * @param context The meshing context containing mesh data and operations
     * @param qualitySettings Quality bounds for the refinement
     * @param exportPrefix Prefix for the debug mesh exports written during refinement
     */
    ShewchukRefiner2D(MeshingContext2D& context,
                      const MeshQualitySettings2D& qualitySettings,
                      std::string exportPrefix = "");

    ~ShewchukRefiner2D();

    /**
     * @brief Run the refinement algorithm until quality goals are met
     *
     * Iteratively refines the mesh by splitting encroached segments and
     * inserting circumcenters until the quality controller is satisfied
     * or the element limit is reached.
     */
    void refine();

    /**
     * @brief Callback type invoked after every boundary segment split.
     *
     * Parameters: (nodeId1, nodeId2, midpointNodeId) — the two original node
     * IDs and the new midpoint node ID. Use this to propagate the split to twin segments
     * (e.g. for periodic boundary conditions via TwinManager2D).
     */
    using BoundarySplitCallback = std::function<void(NodeId nodeId1, NodeId nodeId2, NodeId midpointNodeId)>;

    /**
     * @brief Register a callback that fires after each boundary segment split.
     *
     * The callback is called with the original segment endpoints and the newly
     * inserted midpoint node ID. It is NOT called for interior splits (circumcenter
     * insertion). The callback is called directly after the split succeeds, while
     * the mesh is in a consistent state.
     */
    void setOnBoundarySplit(BoundarySplitCallback callback);

private:
    MeshingContext2D* context_;
    std::unique_ptr<IQualityController2D> qualityController_;

    /**
     * @brief Perform a single refinement step
     * @return true if a refinement was performed, false if mesh is acceptable
     */
    bool refineStep();

    /**
     * @brief Handle an encroached segment by splitting it
     * @param segment The encroached segment to split
     * @return true if the segment was split
     */
    bool handleEncroachedSegment(const CurveSegment& segment);

    /**
     * @brief Handle a poor quality triangle by inserting its circumcenter
     * @param triangle The triangle to refine
     * @param triangleId ID of the triangle to refine
     * @return true if refinement was successful, false otherwise
     */
    bool handlePoorQualityTriangle(const TriangleElement& triangle, ElementId triangleId);

    BoundarySplitCallback onBoundarySplit_;
    size_t exportCounter_ = 0;
    std::unordered_set<ElementId> unrefinableTriangles_;
    std::string exportPrefix_;
};

} // namespace Meshing

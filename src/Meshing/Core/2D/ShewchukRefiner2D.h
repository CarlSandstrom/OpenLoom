#pragma once

#include "GeometryStructures2D.h"
#include "Meshing/Interfaces/IQualityController2D.h"
#include "MeshingContext2D.h"
#include <unordered_set>

namespace Meshing
{

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
     * @brief Construct a Shewchuk refiner
     * @param context The meshing context containing mesh data and operations
     * @param qualityController Quality controller defining acceptable mesh quality
     */
    ShewchukRefiner2D(MeshingContext2D& context,
                      const IQualityController2D& qualityController);

    ~ShewchukRefiner2D();

    /**
     * @brief Run the refinement algorithm until quality goals are met
     *
     * Iteratively refines the mesh by splitting encroached segments and
     * inserting circumcenters until the quality controller is satisfied
     * or the element limit is reached.
     */
    void refine();

private:
    MeshingContext2D* context_;
    const IQualityController2D* qualityController_;

    /**
     * @brief Perform a single refinement step
     * @return true if a refinement was performed, false if mesh is acceptable
     */
    bool refineStep();

    /**
     * @brief Handle an encroached segment by splitting it
     * @param segment The encroached segment to split
     */
    void handleEncroachedSegment(const ConstrainedSegment2D& segment);

    /**
     * @brief Handle a poor quality triangle by inserting its circumcenter
     * @param triangleId ID of the triangle to refine
     * @return true if refinement was successful, false otherwise
     */
    bool handlePoorQualityTriangle(size_t triangleId);

    void exportAndVerifyMesh();

    size_t exportCounter_ = 0;
    std::unordered_set<size_t> unrefinableTriangles_; // Triangles that can't be refined (circumcenter in hole)
};

} // namespace Meshing

#include "Meshing/Core/3D/General/FacetDiscretization2DBuilder.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Topology/SeamCollection.h"
#include "Topology/Surface3D.h"

namespace Meshing
{

FacetDiscretization2DBuilder::FacetDiscretization2DBuilder(const Geometry3D::ISurface3D& surface,
                                                           const Topology3D::Surface3D& topoSurface,
                                                           const Topology3D::SeamCollection& seams,
                                                           const DiscretizationResult3D& discretization3D,
                                                           const std::vector<size_t>& globalPointIndices,
                                                           std::function<size_t(size_t)> pointIdxToNode3DId,
                                                           const Geometry3D::GeometryCollection3D& geometry) :
    surface_(surface),
    topoSurface_(topoSurface),
    seams_(seams),
    discretization3D_(discretization3D),
    globalPointIndices_(globalPointIndices),
    pointIdxToNode3DId_(std::move(pointIdxToNode3DId)),
    geometry_(geometry)
{
}

void FacetDiscretization2DBuilder::build()
{
    buildGlobalToLocalMap();
    buildPoints();
    buildCornerMap();
    buildEdgeMaps();
}

void FacetDiscretization2DBuilder::buildGlobalToLocalMap()
{
    for (size_t localIndex = 0; localIndex < globalPointIndices_.size(); ++localIndex)
    {
        globalToLocal_[globalPointIndices_[localIndex]] = localIndex;
    }
}

void FacetDiscretization2DBuilder::buildPoints()
{
    discretization2D_.points.reserve(globalPointIndices_.size());
    discretization2D_.tParameters.reserve(globalPointIndices_.size());
    discretization2D_.geometryIds.reserve(globalPointIndices_.size());
    localIndexToNode3DId_.reserve(globalPointIndices_.size());

    for (size_t localIndex = 0; localIndex < globalPointIndices_.size(); ++localIndex)
    {
        size_t globalPointIndex = globalPointIndices_[localIndex];
        discretization2D_.points.push_back(surface_.projectPoint(discretization3D_.points[globalPointIndex]));

        // Normalise 3D edge parameters to [0,1] for use with LinearEdge2D.
        const auto& edgeParameters = discretization3D_.edgeParameters[globalPointIndex];
        const auto& geometryIds = discretization3D_.geometryIds[globalPointIndex];
        std::vector<double> normalizedParams;
        normalizedParams.reserve(edgeParameters.size());
        for (size_t k = 0; k < edgeParameters.size(); ++k)
        {
            double t = edgeParameters[k];
            if (k < geometryIds.size())
            {
                const auto* edge3D = geometry_.getEdge(geometryIds[k]);
                if (edge3D)
                {
                    auto [tMin, tMax] = edge3D->getParameterBounds();
                    if (tMax > tMin)
                    {
                        t = (t - tMin) / (tMax - tMin);
                    }
                }
            }
            normalizedParams.push_back(t);
        }
        discretization2D_.tParameters.push_back(std::move(normalizedParams));
        discretization2D_.geometryIds.push_back(discretization3D_.geometryIds[globalPointIndex]);
        localIndexToNode3DId_.push_back(pointIdxToNode3DId_(globalPointIndex));
    }
}

void FacetDiscretization2DBuilder::buildCornerMap()
{
    for (const auto& cornerId : topoSurface_.getCornerIds())
    {
        auto it = discretization3D_.cornerIdToPointIndexMap.find(cornerId);
        if (it != discretization3D_.cornerIdToPointIndexMap.end())
        {
            auto localIt = globalToLocal_.find(it->second);
            if (localIt != globalToLocal_.end())
            {
                discretization2D_.cornerIdToPointIndexMap[cornerId] = localIt->second;
            }
        }
    }
}

void FacetDiscretization2DBuilder::buildEdgeMaps()
{
    auto realCornerToShiftedLocal = processSeamEdges();
    processNonSeamEdges(realCornerToShiftedLocal);
}

FacetDiscretization2DBuilder::ShiftedLocalMaps FacetDiscretization2DBuilder::processSeamEdges()
{
    // Phase 1: Duplicate seam-twin edge points at the period-shifted UV position so the
    // periodic boundary closes correctly in UV space. Returns ShiftedLocalMaps with separate
    // maps for U-shifted and V-shifted copies, which phase 2 uses to close any circular
    // (start == end) edges by selecting the correct shifted endpoint based on UV direction.
    ShiftedLocalMaps shiftedLocals;

    std::vector<std::string> seamTwinEdgeIds;
    for (const auto& edgeId : topoSurface_.getBoundaryEdgeIds())
    {
        if (seams_.isSeamTwin(edgeId))
            seamTwinEdgeIds.push_back(edgeId);
    }

    if (seamTwinEdgeIds.empty())
        return shiftedLocals;

    // Tracks twinStartUV positions that have already been added as local indices.
    // On doubly-periodic surfaces (e.g. torus) both seam twins share the same far corner
    // (e.g. (uMax, vMax)). The second twin must reuse the first twin's local index for that
    // corner rather than inserting a duplicate coincident node, which would cause
    // findConflictingTriangles to return empty (the duplicate lies exactly on all circumcircles).
    std::map<std::pair<double, double>, size_t> closedLoopFarCornerIndex;

    for (const auto& seamId : seamTwinEdgeIds)
    {
        auto it = discretization3D_.edgeIdToPointIndicesMap.find(seamId);
        if (it == discretization3D_.edgeIdToPointIndicesMap.end())
            continue;

        const bool isUSeam =
            seams_.getSeamDirection(seamId) == Topology3D::SeamCollection::SeamDirection::U;

        std::vector<size_t> seamLocalIndices;
        seamLocalIndices.reserve(it->second.size());

        // The twin sequence is the REVERSE of the original seam sequence.
        // For closed-loop seams on doubly-periodic surfaces (e.g. torus), the shared
        // vertex appears at both ends. The first point in the reversed twin sequence
        // corresponds to the original seam's END, whose UV differs from what
        // surface_.projectPoint() would return (projectPoint only returns the principal UV).
        // In that case use the precomputed twinStartUV from SeamCollection directly.
        const auto& seamSeq = it->second;
        const bool isTwinClosedLoop =
            seamSeq.size() > 1 && seamSeq.front() == seamSeq.back();

        const auto twinStartArr = seams_.getTwinStartUV(seamId);
        const Point2D twinStartUV(twinStartArr[0], twinStartArr[1]);

        // Derive the UV shift from the precomputed twinEndUV and the last seam point.
        // twinEndUV is the shifted position of the original seam's start corner (last in twin
        // sequence for closed loops). Using the difference avoids hardcoding +/-period.
        const auto twinEndArr = seams_.getTwinEndUV(seamId);
        const Point2D twinEndUV(twinEndArr[0], twinEndArr[1]);
        const auto seamLastUV =
            surface_.projectPoint(discretization3D_.points[seamSeq.back()]);
        const Point2D uvShift = twinEndUV - seamLastUV;

        for (size_t i = 0; i < seamSeq.size(); ++i)
        {
            size_t globalIdx = seamSeq[i];
            size_t newLocalIdx = discretization2D_.points.size();

            // The first point of a closed-loop twin has a UV different from projectPoint.
            // Use the precomputed twinStartUV (which already includes the period shift).
            const bool isClosedLoopFirstPoint = isTwinClosedLoop && (i == 0);
            if (isClosedLoopFirstPoint)
            {
                // Deduplicate: if another seam twin already created a node at this UV
                // (e.g. both U- and V-seam twins share (uMax,vMax) on a torus), reuse it.
                const auto uvKey = std::make_pair(twinStartUV.x(), twinStartUV.y());
                auto existingIt = closedLoopFarCornerIndex.find(uvKey);
                if (existingIt != closedLoopFarCornerIndex.end())
                {
                    // The far corner is shared by multiple seam twins (e.g. on a torus both
                    // U- and V-seam twins meet at (uMax, vMax)). Add this twin's ID to the
                    // existing entry so splitConstrainedSegment can match it with adjacent
                    // interior seam-twin points that carry only this twin's geometry ID.
                    discretization2D_.geometryIds[existingIt->second].push_back(seamId);
                    seamLocalIndices.push_back(existingIt->second);
                    continue;
                }

                discretization2D_.points.push_back(twinStartUV);
                closedLoopFarCornerIndex[uvKey] = newLocalIdx;
            }
            else
            {
                // Project to UV and shift to the far side of the seam.
                auto uv = surface_.projectPoint(discretization3D_.points[globalIdx]);
                discretization2D_.points.push_back(uv + uvShift);
            }

            // Copy metadata from the original local entry, replacing the original
            // seam edge ID with the seam twin ID so that findCommonGeometryId /
            // splitConstrainedSegment use the shifted UV edge.
            auto origIt = globalToLocal_.find(globalIdx);
            if (origIt != globalToLocal_.end())
            {
                size_t origLocalIdx = origIt->second;
                discretization2D_.tParameters.push_back(discretization2D_.tParameters[origLocalIdx]);

                const std::string& originalSeamId = seams_.getOriginalEdgeId(seamId);
                auto geoIdsCopy = discretization2D_.geometryIds[origLocalIdx];
                for (auto& geoId : geoIdsCopy)
                {
                    if (geoId == originalSeamId)
                        geoId = seamId;
                }
                // The far corner of a closed-loop twin has geoIds from the 3D vertex
                // (e.g. {"v0"}), not from the seam edge. Explicitly add the twin ID so
                // splitConstrainedSegment can match this corner with adjacent interior points.
                if (isClosedLoopFirstPoint)
                    geoIdsCopy.push_back(seamId);
                discretization2D_.geometryIds.push_back(std::move(geoIdsCopy));
            }
            else
            {
                discretization2D_.tParameters.push_back({});
                discretization2D_.geometryIds.push_back({});
            }

            localIndexToNode3DId_.push_back(pointIdxToNode3DId_(globalIdx));
            // Only update shiftedLocals for points that correspond to the twinEndUV side
            // (the last point in the closed-loop twin = original seam start, at twinEndUV).
            // The first point (twinStartUV = far corner) is not used as a shifted endpoint.
            if (!isClosedLoopFirstPoint)
            {
                if (isUSeam)
                    shiftedLocals.uShifted[globalIdx] = newLocalIdx;
                else
                    shiftedLocals.vShifted[globalIdx] = newLocalIdx;
            }
            seamLocalIndices.push_back(newLocalIdx);
        }

        discretization2D_.edgeIdToPointIndicesMap[seamId] = std::move(seamLocalIndices);
    }

    return shiftedLocals;
}

void FacetDiscretization2DBuilder::processNonSeamEdges(const ShiftedLocalMaps& shiftedLocals)
{
    // Phase 2: Map non-seam edges to local indices. Closed-circle edges (start
    // and end share the same 3D global index) redirect their endpoint to the
    // seam-shifted local index so the boundary closes at U/V + period.
    //
    // For doubly-periodic surfaces (torus), there are two seam twins with
    // different shift directions (U and V). The closed-loop seam edge running
    // along U needs the V-shifted endpoint; the one running along V needs the
    // U-shifted endpoint. We detect the edge's UV direction from the second-
    // to-last point relative to the first point to pick the right map.
    for (const auto& edgeId : topoSurface_.getBoundaryEdgeIds())
    {
        if (seams_.isSeamTwin(edgeId))
            continue;

        auto it = discretization3D_.edgeIdToPointIndicesMap.find(edgeId);
        if (it == discretization3D_.edgeIdToPointIndicesMap.end())
            continue;

        const auto& globalSeq = it->second;
        if (globalSeq.empty())
            continue;

        const size_t firstGlobal = globalSeq.front();
        std::vector<size_t> localEdgeIndices;
        localEdgeIndices.reserve(globalSeq.size());

        for (size_t i = 0; i < globalSeq.size(); ++i)
        {
            size_t globalIdx = globalSeq[i];

            const bool isLastPoint = (i == globalSeq.size() - 1);
            const bool isClosedCircle = isLastPoint && (globalIdx == firstGlobal);
            const bool hasShiftedCopy = shiftedLocals.uShifted.count(globalIdx) ||
                                        shiftedLocals.vShifted.count(globalIdx);

            if (isClosedCircle && hasShiftedCopy && globalSeq.size() >= 2)
            {
                // Determine the UV trajectory direction from the second-to-last point.
                // An edge running along V (deltaV >= deltaU) needs the U-shifted endpoint.
                // An edge running along U (deltaU > deltaV)  needs the V-shifted endpoint.
                size_t prevGlobalIdx = globalSeq[globalSeq.size() - 2];
                auto prevIt = globalToLocal_.find(prevGlobalIdx);
                auto startIt = globalToLocal_.find(firstGlobal);

                if (prevIt != globalToLocal_.end() && startIt != globalToLocal_.end())
                {
                    const Point2D& prevUV = discretization2D_.points[prevIt->second];
                    const Point2D& startUV = discretization2D_.points[startIt->second];
                    const double deltaU = std::abs(prevUV.x() - startUV.x());
                    const double deltaV = std::abs(prevUV.y() - startUV.y());

                    // Edge runs along U (deltaU > deltaV) → crosses the U seam → use U-shifted copy
                    // Edge runs along V (deltaV >= deltaU) → crosses the V seam → use V-shifted copy
                    const auto& targetMap =
                        (deltaU > deltaV) ? shiftedLocals.uShifted : shiftedLocals.vShifted;
                    auto shiftedIt = targetMap.find(globalIdx);
                    if (shiftedIt != targetMap.end())
                    {
                        localEdgeIndices.push_back(shiftedIt->second);
                        continue;
                    }
                }
            }

            auto localIt = globalToLocal_.find(globalIdx);
            if (localIt != globalToLocal_.end())
            {
                localEdgeIndices.push_back(localIt->second);
            }
        }

        discretization2D_.edgeIdToPointIndicesMap[edgeId] = std::move(localEdgeIndices);
    }
}

} // namespace Meshing

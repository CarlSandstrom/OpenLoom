#pragma once

#include "Meshing/Core/2D/Periodic/TwinManager2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include <cstddef>

namespace Meshing
{

class MeshingContext2D;

/**
 * @brief Standard implementation of ShewchukRefiner2D::BoundarySplitCallback
 *        for same-context twin-edge synchronisation.
 *
 * When a boundary segment is split during refinement, this class looks up the
 * twin segment in a TwinManager2D, finds and splits it in the same
 * MeshingContext2D, then records the split so both halves stay registered.
 *
 * Usage:
 * @code
 * TwinManager2D twins;
 * twins.registerTwin(n1, n2, m1, m2);
 *
 * BoundarySplitSynchronizer2D sync(context, twins);
 * refiner.setOnBoundarySplit(sync);
 * refiner.refine();
 * @endcode
 *
 * The synchroniser holds non-owning references to both the context and the
 * TwinManager2D, which must outlive the refiner.
 */
class BoundarySplitSynchronizer2D
{
public:
    BoundarySplitSynchronizer2D(MeshingContext2D& context, TwinManager2D& twinManager);

    /**
     * @brief Callable matching ShewchukRefiner2D::BoundarySplitCallback.
     *
     * Looks up the twin of segment (node0, node1), splits it, and records the split.
     * If (node0, node1) has no registered twin the call is a no-op.
     */
    void operator()(size_t node0, size_t node1, size_t midpointNode);

private:
    MeshingContext2D* context_;
    TwinManager2D* twinManager_;
};

} // namespace Meshing

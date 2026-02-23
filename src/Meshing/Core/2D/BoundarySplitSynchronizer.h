#pragma once

#include "Common/TwinManager.h"
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
 * twin segment in a TwinManager, finds and splits it in the same
 * MeshingContext2D, then records the split so both halves stay registered.
 *
 * Usage:
 * @code
 * TwinManager twins;
 * twins.registerTwin(n1, n2, m1, m2);
 *
 * BoundarySplitSynchronizer sync(context, twins);
 * refiner.setOnBoundarySplit(sync);
 * refiner.refine();
 * @endcode
 *
 * The synchroniser holds non-owning references to both the context and the
 * TwinManager, which must outlive the refiner.
 *
 * For 3D use (twins on different faces / different MeshingContext2D instances)
 * the FacetTriangulationManager implements its own cross-context callback.
 */
class BoundarySplitSynchronizer
{
public:
    BoundarySplitSynchronizer(MeshingContext2D& context, TwinManager& twinManager);

    /**
     * @brief Callable matching ShewchukRefiner2D::BoundarySplitCallback.
     *
     * Looks up the twin of segment (n1, n2), splits it, and records the split.
     * If (n1, n2) has no registered twin the call is a no-op.
     */
    void operator()(size_t n1, size_t n2, size_t mid);

private:
    MeshingContext2D* context_;
    TwinManager*      twinManager_;
};

} // namespace Meshing

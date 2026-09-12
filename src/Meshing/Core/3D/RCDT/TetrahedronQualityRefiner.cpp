#include "Meshing/Core/3D/RCDT/TetrahedronQualityRefiner.h"

#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Core/3D/RCDT/RCDTPointInserter.h"
#include "Meshing/Core/3D/RCDT/RCDTTetQualityController.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

namespace Meshing
{

TetrahedronQualityRefiner::TetrahedronQualityRefiner(MeshingContext3D& context,
                                                     const RestrictedTriangulation& restrictedTriangulation,
                                                     const RCDTTetQualityController& tetrahedronQualityController,
                                                     double circumradiusToShortestEdgeRatio,
                                                     double minimumEdgeLength) :
    context_(&context),
    restrictedTriangulation_(&restrictedTriangulation),
    tetrahedronQualityController_(&tetrahedronQualityController),
    circumradiusToShortestEdgeRatio_(circumradiusToShortestEdgeRatio),
    minimumEdgeLength_(minimumEdgeLength)
{
}

bool TetrahedronQualityRefiner::refineNext(RCDTPointInserter& pointInserter)
{
    const auto& meshData = context_->getMeshData();

    // AmbientTetrahedronRemover only runs after refinement, so the mesh still
    // contains ambient tetrahedra: those touching the bounding tetrahedron's
    // corners, and those filling holes in the domain. Neither is output, and
    // both are skinny by nature, so refining them would spend iterations on
    // meaningless circumcenters. AmbientTetrahedronClassifier finds both.
    const auto ambientTetIds = AmbientTetrahedronClassifier::classify(meshData, *restrictedTriangulation_);

    const auto skinnyTetIds =
        context_->getOperations().getQueries().findSkinnyTetrahedra(circumradiusToShortestEdgeRatio_);

    const ElementGeometry3D elementGeometry(meshData);

    for (const size_t tetId : skinnyTetIds)
    {
        if (unrefinableTetrahedra_.count(tetId))
            continue;

        const auto* element = meshData.getElement(tetId);
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
        if (!tet)
            continue;

        if (ambientTetIds.contains(tetId))
            continue;

        // Size floor: a degenerate tetrahedron is left unrefined.
        if (tetrahedronQualityController_->isTetrahedronTooSmall(*tet))
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        const auto circumsphere = elementGeometry.computeCircumscribingSphere(*tet);
        if (!circumsphere)
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        // As a tetrahedron flattens, its circumcenter recedes toward infinity
        // (real geometry, not numerical error), so inserting it can land far
        // outside the mesh -- radius 126 was measured on a unit box -- and
        // refinement never converges. The bound uses minimumEdgeLength_, the
        // mesh's own scale, rather than this tetrahedron's possibly tiny size.
        // This is the sliver limitation in the class doc.
        constexpr double MAX_CIRCUMRADIUS_TO_MIN_EDGE_LENGTH_RATIO = 100.0;
        if (circumsphere->radius > MAX_CIRCUMRADIUS_TO_MIN_EDGE_LENGTH_RATIO * minimumEdgeLength_)
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        const Point3D& circumcenter = circumsphere->center;

        // Interior point: no geometryIds, matching insertVertexBowyerWatson's
        // convention for a non-boundary node.
        if (pointInserter.tryInsert(circumcenter, {}))
            return true;
        unrefinableTetrahedra_.insert(tetId);
    }

    return false;
}

} // namespace Meshing

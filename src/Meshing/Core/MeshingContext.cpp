#include "MeshingContext.h"

#include "../../Geometry/Base/GeometryCollection.h"
#include "../../Topology/Topology.h"
#include "../Data/MeshConnectivity.h"
#include "../Data/MeshData.h"
#include "../Data/MeshOperations.h"

namespace Meshing
{

MeshingContext::MeshingContext(const Geometry::GeometryCollection& geometry,
                               const Topology::Topology& topology) :
    geometry_(geometry), topology_(topology)
{
    ensureInitialized_();
}

MeshingContext::~MeshingContext() = default;

MeshData& MeshingContext::getMeshData()
{
    ensureInitialized_();
    return *meshData_;
}

MeshConnectivity& MeshingContext::getConnectivity()
{
    ensureInitialized_();
    return *connectivity_;
}

MeshOperations& MeshingContext::getOperations()
{
    ensureInitialized_();
    return *operations_;
}

void MeshingContext::rebuildConnectivity()
{
    ensureInitialized_();
    connectivity_->rebuildConnectivity();
}

void MeshingContext::clearMesh()
{
    // Recreate fresh containers
    meshData_ = std::make_unique<MeshData>();
    connectivity_ = std::make_unique<MeshConnectivity>(*meshData_);
    operations_ = std::make_unique<MeshOperations>(*meshData_);
    operations_->setConnectivity(connectivity_.get());
}

void MeshingContext::ensureInitialized_()
{
    if (!meshData_)
    {
        meshData_ = std::make_unique<MeshData>();
    }
    if (!connectivity_)
    {
        connectivity_ = std::make_unique<MeshConnectivity>(*meshData_);
    }
    if (!operations_)
    {
        operations_ = std::make_unique<MeshOperations>(*meshData_);
        operations_->setConnectivity(connectivity_.get());
    }
}

} // namespace Meshing

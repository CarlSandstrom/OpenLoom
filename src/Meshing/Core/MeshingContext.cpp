#include "MeshingContext.h"

#include "../../Geometry/Base/GeometryCollection3D.h"
#include "../../Topology/Topology3D.h"
#include "../Data/MeshConnectivity.h"
#include "../Data/MeshData.h"
#include "../Data/MeshOperations.h"

namespace Meshing
{

MeshingContext3D::MeshingContext3D(const Geometry3D::GeometryCollection3D& geometry,
                                   const Topology3D::Topology3D& topology) :
    geometry_(geometry), topology_(topology)
{
    ensureInitialized_();
}

MeshingContext3D::~MeshingContext3D() = default;

MeshData& MeshingContext3D::getMeshData()
{
    ensureInitialized_();
    return *meshData_;
}

MeshConnectivity& MeshingContext3D::getConnectivity()
{
    ensureInitialized_();
    return *connectivity_;
}

MeshOperations& MeshingContext3D::getOperations()
{
    ensureInitialized_();
    return *operations_;
}

void MeshingContext3D::rebuildConnectivity()
{
    ensureInitialized_();
    connectivity_->rebuildConnectivity();
}

void MeshingContext3D::clearMesh()
{
    // Recreate fresh containers
    meshData_ = std::make_unique<MeshData>();
    connectivity_ = std::make_unique<MeshConnectivity>(*meshData_);
    operations_ = std::make_unique<MeshOperations>(*meshData_);
    operations_->setConnectivity(connectivity_.get());
}

void MeshingContext3D::ensureInitialized_()
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

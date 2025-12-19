#include "MeshingContext3D.h"

#include "../../Geometry/Base/GeometryCollection3D.h"
#include "../../Topology/Topology3D.h"
#include "../Data/MeshConnectivity.h"
#include "../Data/MeshData3D.h"
#include "../Data/MeshMutator3D.h"

namespace Meshing
{

MeshingContext3D::MeshingContext3D(const Geometry3D::GeometryCollection3D& geometry,
                                   const Topology3D::Topology3D& topology) :
    geometry_(geometry), topology_(topology)
{
    ensureInitialized();
}

MeshingContext3D::~MeshingContext3D() = default;

MeshData3D& MeshingContext3D::getMeshData()
{
    ensureInitialized();
    return *meshData_;
}

MeshConnectivity& MeshingContext3D::getConnectivity()
{
    ensureInitialized();
    return *connectivity_;
}

MeshMutator3D& MeshingContext3D::getOperations()
{
    ensureInitialized();
    return *operations_;
}

void MeshingContext3D::rebuildConnectivity()
{
    ensureInitialized();
    connectivity_->rebuildConnectivity();
}

void MeshingContext3D::clearMesh()
{
    // Recreate fresh containers
    meshData_ = std::make_unique<MeshData3D>();
    connectivity_ = std::make_unique<MeshConnectivity>(*meshData_);
    operations_ = std::make_unique<MeshMutator3D>(*meshData_);
    operations_->setConnectivity(connectivity_.get());
}

void MeshingContext3D::ensureInitialized()
{
    if (!meshData_)
    {
        meshData_ = std::make_unique<MeshData3D>();
    }
    if (!connectivity_)
    {
        connectivity_ = std::make_unique<MeshConnectivity>(*meshData_);
    }
    if (!operations_)
    {
        operations_ = std::make_unique<MeshMutator3D>(*meshData_);
        operations_->setConnectivity(connectivity_.get());
    }
}

} // namespace Meshing

#include "Meshing/Core/3D/General/MeshingContext3D.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology/Topology3D.h"

namespace Meshing
{

MeshingContext3D::MeshingContext3D(const Geometry3D::GeometryCollection3D& geometry,
                                   const Topology3D::Topology3D& topology) :
    geometry_(&geometry), topology_(&topology)
{
    ensureInitialized();
}

MeshingContext3D::MeshingContext3D() :
    geometry_(nullptr), topology_(nullptr)
{
    ensureInitialized();
}

MeshingContext3D::~MeshingContext3D() = default;

MeshingContext3D::MeshingContext3D(MeshingContext3D&&) noexcept = default;
MeshingContext3D& MeshingContext3D::operator=(MeshingContext3D&&) noexcept = default;

MeshData3D& MeshingContext3D::getMeshData()
{
    ensureInitialized();
    return *meshData_;
}

const MeshData3D& MeshingContext3D::getMeshData() const
{
    return *meshData_;
}

MeshConnectivity& MeshingContext3D::getConnectivity()
{
    ensureInitialized();
    return *connectivity_;
}

MeshMutator3D& MeshingContext3D::getMutator()
{
    ensureInitialized();
    return *meshMutator_;
}

MeshOperations3D& MeshingContext3D::getOperations()
{
    ensureInitialized();
    return *meshOperations_;
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
    meshMutator_ = std::make_unique<MeshMutator3D>(*meshData_);
    meshMutator_->setConnectivity(connectivity_.get());
    meshOperations_ = std::make_unique<MeshOperations3D>(*meshData_);
}

// ========== Private Methods ==========

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
    if (!meshMutator_)
    {
        meshMutator_ = std::make_unique<MeshMutator3D>(*meshData_);
        meshMutator_->setConnectivity(connectivity_.get());
    }
    if (!meshOperations_)
    {
        meshOperations_ = std::make_unique<MeshOperations3D>(*meshData_);
    }
}

} // namespace Meshing
